#!/usr/bin/env bash

set -euo pipefail

# 这个脚本在 NX 上运行，用来把 NX 的 Wi-Fi 切到手机热点。
# 它不会修改雷达网口，只负责远程控制这条链路。
# 切换成功后，Windows 台式机也连到同一个热点，再用配套的 PowerShell 脚本扫描 NX 的新 IP。

SSID="${1:-}"
PASSWORD="${2:-}"

pick_wifi_device() {
    nmcli -t -f DEVICE,TYPE,STATE device status | awk -F: '$2=="wifi"{print $1; exit}'
}

WIFI_DEV="$(pick_wifi_device || true)"

if [[ -z "$WIFI_DEV" ]]; then
    echo "[错误] 没找到 Wi-Fi 网卡，无法连接手机热点。" >&2
    exit 1
fi

if [[ -z "$SSID" ]]; then
    read -r -p "请输入手机热点名称 SSID: " SSID
fi

if [[ -z "$PASSWORD" ]]; then
    read -r -s -p "请输入手机热点密码: " PASSWORD
    echo
fi

if [[ -z "$SSID" || -z "$PASSWORD" ]]; then
    echo "[错误] 热点名称或密码为空，已取消。" >&2
    exit 1
fi

echo "[信息] 当前 Wi-Fi 设备: $WIFI_DEV"
echo "[信息] 正在连接手机热点: $SSID"

nmcli device wifi connect "$SSID" password "$PASSWORD" ifname "$WIFI_DEV"

sleep 3

IP_LINE="$(nmcli -g IP4.ADDRESS device show "$WIFI_DEV" | head -n 1 || true)"
GATEWAY="$(nmcli -g IP4.GATEWAY device show "$WIFI_DEV" | head -n 1 || true)"

{
    echo "热点名称: $SSID"
    echo "Wi-Fi 网卡: $WIFI_DEV"
    echo "主机名: $(hostname)"
    echo "IPv4 地址: ${IP_LINE:-未获取到}"
    echo "网关: ${GATEWAY:-未获取到}"
    echo "当前时间: $(date '+%F %T')"
} | tee "$HOME/nx_hotspot_info.txt"

echo
echo "[完成] NX 已尝试切换到手机热点。"
echo "[下一步] 请让台式机也连到同一个手机热点，然后在 Windows 上运行："
echo "  powershell -ExecutionPolicy Bypass -File D:/repos/slam-drone/catkin_ws/tools/push_latest_to_nx.ps1"
