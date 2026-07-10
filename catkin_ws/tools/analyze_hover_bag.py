#!/usr/bin/env python3

import argparse
import math
import re
import statistics
import sys
from pathlib import Path

try:
    import rosbag
except Exception as exc:  # pragma: no cover - only runs on ROS machine
    print("[错误] 无法 import rosbag。请在 NX/ROS 环境中运行，并先 source /opt/ros/noetic/setup.bash。", file=sys.stderr)
    print(f"[错误] {exc}", file=sys.stderr)
    sys.exit(1)


def mean(values):
    return statistics.fmean(values) if values else float("nan")


def median(values):
    return statistics.median(values) if values else float("nan")


def stdev(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def percentile(values, pct):
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = (len(ordered) - 1) * pct / 100.0
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - idx) + ordered[hi] * (idx - lo)


def fmt(value, digits=3):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "n/a"
    return f"{value:.{digits}f}"


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def parse_yaml_scalar(path, key):
    if not path or not Path(path).expanduser().exists():
        return None
    text = Path(path).expanduser().read_text(encoding="utf-8", errors="ignore")
    match = re.search(rf"^\s*{re.escape(key)}\s*:\s*([-+]?\d+(?:\.\d+)?)", text, re.MULTILINE)
    if not match:
        return None
    return float(match.group(1))


def mode_durations(state_samples, bag_duration):
    if not state_samples:
        return []
    result = {}
    for idx, (t, _armed, mode) in enumerate(state_samples):
        t_next = state_samples[idx + 1][0] if idx + 1 < len(state_samples) else bag_duration
        result[mode] = result.get(mode, 0.0) + max(0.0, t_next - t)
    return sorted(result.items(), key=lambda item: item[1], reverse=True)


def nearest_series_diff(primary, secondary, max_dt=0.05):
    if not primary or not secondary:
        return []
    diffs = []
    j = 0
    for t, z, *_ in primary:
        while j + 1 < len(secondary) and abs(secondary[j + 1][0] - t) < abs(secondary[j][0] - t):
            j += 1
        dt = abs(secondary[j][0] - t)
        if dt <= max_dt:
            diffs.append(secondary[j][1] - z)
    return diffs


def read_bag(bag_path):
    bag = rosbag.Bag(str(bag_path), "r")
    start = bag.get_start_time()
    end = bag.get_end_time()
    topics_present = set(bag.get_type_and_topic_info().topics.keys())

    data = {
        "duration": end - start,
        "topics": topics_present,
        "state": [],
        "odom": [],
        "control_odom": [],
        "local_pose": [],
        "vision_pose": [],
        "local_vel": [],
        "debug": [],
        "att_sp": [],
        "rc_in": [],
        "rc_out": [],
        "battery": [],
    }

    wanted = {
        "/mavros/state",
        "/Odometry",
        "/fastlio_odom_with_velocity",
        "/mavros/local_position/pose",
        "/mavros/vision_pose/pose",
        "/mavros/local_position/velocity_local",
        "/debugPx4ctrl",
        "/mavros/setpoint_raw/attitude",
        "/mavros/rc/in",
        "/mavros/rc/out",
        "/mavros/battery",
    }

    for topic, msg, stamp in bag.read_messages(topics=list(wanted)):
        t = stamp.to_sec() - start

        if topic == "/mavros/state":
            data["state"].append((t, bool(msg.armed), str(msg.mode)))

        elif topic in ("/Odometry", "/fastlio_odom_with_velocity"):
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            key = "control_odom" if topic == "/fastlio_odom_with_velocity" else "odom"
            data[key].append((t, float(p.z), float(v.z)))

        elif topic == "/mavros/local_position/pose":
            data["local_pose"].append((t, float(msg.pose.position.z)))

        elif topic == "/mavros/vision_pose/pose":
            data["vision_pose"].append((t, float(msg.pose.position.z)))

        elif topic == "/mavros/local_position/velocity_local":
            data["local_vel"].append((t, float(msg.twist.linear.z)))

        elif topic == "/debugPx4ctrl":
            data["debug"].append((
                t,
                float(getattr(msg, "des_thr", float("nan"))),
                float(getattr(msg, "des_v_z", float("nan"))),
                float(getattr(msg, "des_a_z", float("nan"))),
                float(getattr(msg, "hover_percentage", float("nan"))),
                float(getattr(msg, "thr_scale_compensate", float("nan"))),
            ))

        elif topic == "/mavros/setpoint_raw/attitude":
            data["att_sp"].append((t, float(msg.thrust)))

        elif topic == "/mavros/rc/in":
            channels = list(msg.channels)
            while len(channels) < 8:
                channels.append(0)
            data["rc_in"].append((t, channels))

        elif topic == "/mavros/rc/out":
            channels = [float(x) for x in msg.channels]
            motors = [x for x in channels[:4] if 900.0 <= x <= 2200.0]
            if motors:
                avg_pwm = mean(motors)
                norm = clamp((avg_pwm - 1000.0) / 1000.0, 0.0, 1.0)
                data["rc_out"].append((t, avg_pwm, norm))

        elif topic == "/mavros/battery":
            data["battery"].append((t, float(msg.voltage), float(getattr(msg, "percentage", float("nan")))))

    bag.close()
    return data


def pick_hover_window(odom, target_z):
    if not odom:
        return None, []

    first_t = odom[0][0]
    initial = [z for t, z, _vz in odom if t <= first_t + 2.0]
    home_z = median(initial if initial else [odom[0][1]])

    airborne = [sample for sample in odom if sample[1] > home_z + 0.15 or abs(sample[2]) > 0.20]
    if not airborne:
        return (home_z, odom[0][0], odom[-1][0]), odom

    t0 = airborne[0][0]
    t1 = airborne[-1][0]
    duration = max(0.1, t1 - t0)
    hover_start = t0 + max(5.0, duration * 0.35)
    min_z = home_z + max(0.20, target_z * 0.35)
    hover = [sample for sample in odom if sample[0] >= hover_start and sample[1] >= min_z]

    if len(hover) < 20:
        hover_start = t0 + duration * 0.50
        hover = [sample for sample in odom if sample[0] >= hover_start]

    if len(hover) < 20:
        hover = odom[-min(len(odom), 200):]
        hover_start = hover[0][0] if hover else t0

    return (home_z, hover_start, t1), hover


def filter_by_window(series, t0, t1, value_index=1):
    values = []
    for item in series:
        if t0 <= item[0] <= t1:
            value = item[value_index]
            if isinstance(value, float) and not math.isnan(value):
                values.append(value)
    return values


def normalized_rc_channel(pwm):
    value = (float(pwm) - 1500.0) / 500.0
    dead_zone = 0.25
    if value > dead_zone:
        return (value - dead_zone) / (1.0 - dead_zone)
    if value < -dead_zone:
        return (value + dead_zone) / (1.0 - dead_zone)
    return 0.0


def analyze(args):
    bag_path = Path(args.bag).expanduser()
    data = read_bag(bag_path)

    current_hover = args.current_hover
    if current_hover is None:
        current_hover = parse_yaml_scalar(args.config, "hover_percentage")
    current_kp2 = parse_yaml_scalar(args.config, "Kp2")
    current_kv2 = parse_yaml_scalar(args.config, "Kv2")

    odom_source = "/fastlio_odom_with_velocity" if data["control_odom"] else "/Odometry"
    odom_series = data["control_odom"] if data["control_odom"] else data["odom"]

    window, hover = pick_hover_window(odom_series, args.target_z)
    home_z, hover_start, hover_end = window if window else (float("nan"), 0.0, data["duration"])

    z_values = [z for _t, z, _vz in hover]
    vz_values = [vz for _t, _z, vz in hover]
    abs_vz = [abs(vz) for vz in vz_values]

    z_mean = mean(z_values)
    z_std = stdev(z_values)
    z_min = min(z_values) if z_values else float("nan")
    z_max = max(z_values) if z_values else float("nan")
    z_p2p = z_max - z_min if z_values else float("nan")
    vz_p95 = percentile(abs_vz, 95)

    local_diff = nearest_series_diff(hover, data["local_pose"])
    vision_diff = nearest_series_diff(hover, data["vision_pose"])

    debug_thr = filter_by_window(data["debug"], hover_start, hover_end, 1)
    debug_thr = [x for x in debug_thr if 0.05 <= x <= 0.95]
    att_thr = filter_by_window(data["att_sp"], hover_start, hover_end, 1)
    att_thr = [x for x in att_thr if 0.05 <= x <= 0.95]
    rc_out_norm = filter_by_window(data["rc_out"], hover_start, hover_end, 2)
    rc_out_pwm = filter_by_window(data["rc_out"], hover_start, hover_end, 1)

    thrust_source = None
    observed_hover = None
    if len(debug_thr) >= 20:
        thrust_source = "/debugPx4ctrl des_thr"
        observed_hover = median(debug_thr)
    elif len(att_thr) >= 20:
        thrust_source = "/mavros/setpoint_raw/attitude thrust"
        observed_hover = median(att_thr)
    elif len(rc_out_norm) >= 20:
        thrust_source = "/mavros/rc/out PWM approx"
        observed_hover = median(rc_out_norm)

    next_hover = None
    if current_hover is not None and observed_hover is not None:
        next_hover = current_hover + clamp(observed_hover - current_hover, -0.03, 0.03)

    rc_thr_norms = []
    rc_mode_pwms = []
    rc_gear_pwms = []
    for t, channels in data["rc_in"]:
        if hover_start <= t <= hover_end and len(channels) >= 6:
            rc_thr_norms.append(normalized_rc_channel(channels[2]))
            rc_mode_pwms.append(channels[4])
            rc_gear_pwms.append(channels[5])

    battery_volt = filter_by_window(data["battery"], hover_start, hover_end, 1)

    lines = []
    lines.append("PX4/px4ctrl 悬停 bag 分析")
    lines.append("=" * 32)
    lines.append(f"bag: {bag_path}")
    lines.append(f"duration: {fmt(data['duration'], 1)} s")
    lines.append(f"target_z: {fmt(args.target_z)} m")
    lines.append(f"topics_recorded: {len(data['topics'])}")
    lines.append("")

    lines.append("[模式]")
    durations = mode_durations(data["state"], data["duration"])
    if durations:
        for mode, dur in durations[:6]:
            lines.append(f"- {mode}: {fmt(dur, 1)} s")
    else:
        lines.append("- 没有 /mavros/state，无法判断 PX4 模式。")
    lines.append("")

    lines.append("[高度]")
    lines.append(f"- 分析使用的里程计: {odom_source}")
    lines.append(f"- 起飞前 Odometry z 基准: {fmt(home_z)} m")
    lines.append(f"- 分析窗口: {fmt(hover_start, 1)} s 到 {fmt(hover_end, 1)} s, 样本数 {len(hover)}")
    lines.append(f"- z mean/std/min/max/p2p: {fmt(z_mean)} / {fmt(z_std)} / {fmt(z_min)} / {fmt(z_max)} / {fmt(z_p2p)} m")
    lines.append(f"- |vz| 95%: {fmt(vz_p95)} m/s")
    if local_diff:
        lines.append(f"- /mavros/local_position/pose.z - /Odometry.z: mean {fmt(mean(local_diff))} m, std {fmt(stdev(local_diff))} m")
    if vision_diff:
        lines.append(f"- /mavros/vision_pose/pose.z - /Odometry.z: mean {fmt(mean(vision_diff))} m, std {fmt(stdev(vision_diff))} m")
    lines.append("")

    lines.append("[油门/输出]")
    if observed_hover is not None:
        lines.append(f"- 观测到的悬停输出来源: {thrust_source}")
        lines.append(f"- 观测悬停比例 median: {fmt(observed_hover)}")
    else:
        lines.append("- 没有可用的 des_thr/setpoint thrust/rc_out，无法直接估计悬停油门。")
    if current_hover is not None:
        lines.append(f"- 当前 ctrl_param_fpv.yaml hover_percentage: {fmt(current_hover)}")
    if next_hover is not None:
        lines.append(f"- 建议下一次 hover_percentage: {fmt(next_hover)}  （单次最多改 0.03，避免越调越炸）")
    if rc_out_pwm:
        lines.append(f"- PX4 输出 PWM 均值 median: {fmt(median(rc_out_pwm), 1)} us")
    if battery_volt:
        lines.append(f"- 电池电压 median/min: {fmt(median(battery_volt), 2)} / {fmt(min(battery_volt), 2)} V")
    lines.append("")

    lines.append("[遥控器]")
    if rc_thr_norms:
        thr_med = median(rc_thr_norms)
        thr_abs95 = percentile([abs(v) for v in rc_thr_norms], 95)
        drift_speed = thr_med * 0.35
        lines.append(f"- 油门通道归一化 median: {fmt(thr_med)}，|油门|95%: {fmt(thr_abs95)}")
        lines.append(f"- 按当前 max_manual_vel=0.35 估算，AUTO_HOVER 中油门杆会造成约 {fmt(drift_speed)} m/s 的目标高度漂移。")
        lines.append(f"- mode/gear PWM median: {fmt(median(rc_mode_pwms), 0)} / {fmt(median(rc_gear_pwms), 0)}")
    else:
        lines.append("- 没有 /mavros/rc/in，无法检查油门杆是否居中。")
    lines.append("")

    lines.append("[判断]")
    if not odom_series:
        lines.append("- 严重问题：没有可用里程计。px4ctrl 和 FAST-LIO2 链路不能继续飞。")
    elif z_p2p > 0.50 or vz_p95 > 0.60:
        lines.append("- 高度波动偏大。先确认 hover_percentage 和 FAST-LIO2/PX4 高度估计，再继续 1m 自由飞。")
    elif z_std > 0.12:
        lines.append("- 高度标准差偏大，属于需要继续调参的状态。")
    else:
        lines.append("- 这个窗口内高度波动看起来可以进入下一轮小高度测试。")

    if rc_thr_norms and percentile([abs(v) for v in rc_thr_norms], 95) > 0.05:
        lines.append("- 油门杆在悬停期间没有稳定居中。px4ctrl 的 AUTO_HOVER 会把它当成目标高度微调，必须先修正遥控器中位/死区。")

    if observed_hover is not None and current_hover is not None and abs(observed_hover - current_hover) > 0.04:
        lines.append("- 当前 hover_percentage 和观测值差得比较多，优先调 hover_percentage，不要先大幅改 Kp/Kv。")
    elif current_kp2 is not None and current_kv2 is not None and (z_p2p > 0.45 or vz_p95 > 0.55):
        kp2_next = max(0.8, current_kp2 * 0.9)
        kv2_next = min(2.2, current_kv2 * 1.08)
        lines.append(f"- 若 hover_percentage 已接近观测值，下一轮可试 Kp2={fmt(kp2_next, 2)}, Kv2={fmt(kv2_next, 2)}，一次只改一小步。")

    if "OFFBOARD" in {mode for mode, _dur in durations}:
        lines.append("- 这个包包含 OFFBOARD：它反映 px4ctrl 控制结果。退出 OFFBOARD 后会回到进入 OFFBOARD 前记录的 PX4 模式，或受 PX4 offboard failsafe 参数影响。")
    else:
        lines.append("- 这个包不含 OFFBOARD：它适合判断 PX4 原生定点/定高和外部视觉高度是否稳定。")

    report = "\n".join(lines) + "\n"
    print(report)

    out_dir = Path(args.output_dir).expanduser() if args.output_dir else bag_path.parent / "analysis" / bag_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "hover_analysis.txt").write_text(report, encoding="utf-8")

    snippet_lines = [
        "# Suggested snippet from analyze_hover_bag.py",
        "# Review manually before copying into ctrl_param_fpv.yaml.",
        "max_manual_vel: 0.35",
        "auto_takeoff_land:",
        "    takeoff_height: 0.6",
        "    takeoff_land_speed: 0.15",
        "thrust_model:",
    ]
    if next_hover is not None:
        snippet_lines.append(f"    hover_percentage: {next_hover:.3f}")
    elif observed_hover is not None:
        snippet_lines.append(f"    hover_percentage: {observed_hover:.3f}")
    else:
        snippet_lines.append("    hover_percentage: 0.42")
    if current_kp2 is not None and current_kv2 is not None:
        snippet_lines.extend([
            "gain:",
            f"    Kp2: {current_kp2:.2f}",
            f"    Kv2: {current_kv2:.2f}",
        ])
    (out_dir / "suggested_ctrl_param_snippet.yaml").write_text("\n".join(snippet_lines) + "\n", encoding="utf-8")

    print(f"[完成] 分析结果已写入: {out_dir}")


def main():
    parser = argparse.ArgumentParser(description="Analyze hover rosbag for PX4/px4ctrl altitude tuning.")
    parser.add_argument("bag", help="Path to .bag file")
    parser.add_argument("--target-z", type=float, default=1.0, help="Expected hover height above takeoff point, meters")
    parser.add_argument("--current-hover", type=float, default=None, help="Current hover_percentage value")
    parser.add_argument("--config", default="~/catkin_ws/src/px4ctrl/config/ctrl_param_fpv.yaml", help="ctrl_param_fpv.yaml path")
    parser.add_argument("--output-dir", default=None, help="Directory to write analysis files")
    args = parser.parse_args()
    analyze(args)


if __name__ == "__main__":
    main()
