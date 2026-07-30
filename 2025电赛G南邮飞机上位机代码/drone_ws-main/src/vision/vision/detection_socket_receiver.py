#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msg_tool.msg import Color
from std_msgs.msg import Bool
import socket
import json
import threading
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math

class DetectionReceiver(Node):
    def __init__(self):
        """
        节点初始化函数
        """
        super().__init__('detection_receiver')
        
        # 定义服务质量（QoS）配置，保证消息可靠传输
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
        
        # 创建发布者，用于发布目标信息
        self.pub = self.create_publisher(Color, '/target', qos)
        # 创建订阅者，用于接收机器人是否到达目标的状态
        self.arrived_sub = self.create_subscription(Bool, '/processed_status', self.arrived_callback, qos)

        # --- 相机内参矩阵K ---
        self.cam_fx = 788.0  # x轴焦距
        self.cam_fy = 789.0  # y轴焦距
        self.cam_cx = 288.11 # 光心x坐标
        self.cam_cy = 301.57 # 光心y坐标
        self.height = 1.1   # 相机离地高度 (Z)

        # --- 追踪参数 ---
        self.alpha = 0.7  # 指数移动平均的平滑因子，用于坐标滤波
        self.required_frames_for_detection = 1  # 判定为“稳定检测”所需的帧数，设为1以实现快速响应
        self.color_filter_window_size = 3     # 颜色滤波的滑动窗口大小，保证颜色数据稳定
        self.track_timeout = 1.0              # 目标被视为“丢失”的超时时间（秒）

        # --- 状态管理 ---
        self.tracked_objects = {}  # 存储当前追踪的所有目标信息 {id: track_data}
        self.completed_target_ids = set()  # 存储已完成处理的目标ID
        self.currently_published_target_id = None  # 当前正在发布的目标ID
        
        # 新增：用于“锁定”目标，解决目标丢失后重新追踪的问题
        self.last_known_target_id = None

        # --- Socket服务器，用于从外部接收检测数据 ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 允许地址重用
        self.sock.bind(('0.0.0.0', 9000)) # 绑定到所有网络接口的9000端口
        self.sock.listen(1)
        # 在一个独立的线程中运行socket连接接收循环
        threading.Thread(target=self.accept_loop, daemon=True).start()
        self.get_logger().info('智能目标发布节点已启动，坐标系已校正。')

    def select_and_publish_target(self):
        """
        选择最合适的目标并发布。
        这是节点的核心逻辑。
        """
        self.cleanup_stale_tracks() # 首先，清理掉长时间未见到的目标
        
        selected_target_info = None

        # --- 修改后的目标锁定逻辑 ---
        # 优先级1：尝试重新获取之前“锁定”的目标（如果它再次可见）
        if self.last_known_target_id and self.last_known_target_id in self.tracked_objects:
            # 确保这个锁定的目标没有被标记为“已完成”
            if self.last_known_target_id not in self.completed_target_ids:
                selected_target_info = {'id': self.last_known_target_id, 'track': self.tracked_objects[self.last_known_target_id]}
                self.get_logger().debug(f"重新锁定目标 ID: {self.last_known_target_id}")

        # 优先级2：如果没有锁定的目标，或者锁定的目标当前不可见，则寻找一个最近的新目标
        if not selected_target_info:
            candidate_targets = []
            for obj_id, track in self.tracked_objects.items():
                # 只在未完成的目标里选择
                if obj_id not in self.completed_target_ids:
                    # 计算目标像素坐标与相机光心的距离的平方
                    dist_sq = (track['filtered_u'] - self.cam_cx)**2 + (track['filtered_v'] - self.cam_cy)**2
                    candidate_targets.append({'id': obj_id, 'dist_sq': dist_sq, 'track': track})

            if candidate_targets:
                # 按距离排序，选择最近的一个作为新目标
                candidate_targets.sort(key=lambda x: x['dist_sq'])
                selected_target_info = candidate_targets[0]
                # “锁定”这个新目标
                self.last_known_target_id = selected_target_info['id']
                self.get_logger().info(f"锁定新目标: ID {self.last_known_target_id}")

        # --- 发布消息 ---
        if selected_target_info:
            track_data = selected_target_info['track']
            self.currently_published_target_id = selected_target_info['id']
            
            u_filtered = track_data['filtered_u']
            v_filtered = track_data['filtered_v']

            # 应用相机模型公式，将像素坐标转换为机器人世界坐标系下的坐标
            real_x = self.height * (u_filtered - self.cam_cx) / self.cam_fx
            real_y = self.height * (v_filtered - self.cam_cy) / self.cam_fy
            
            # 组装要发布的消息
            msg = Color()
            msg.color = track_data['color'] # 使用滤波后的稳定颜色
            msg.delta_x = -real_y + 0.06    # 坐标系转换和偏移校准
            msg.delta_y = -real_x           # 坐标系转换
            msg.detected = self.is_stably_detected(track_data) # 判断是否稳定检测到
            
            self.pub.publish(msg)
            self.get_logger().debug(f"发布目标 ID: {self.currently_published_target_id}, 颜色: {msg.color}, DeltaX: {msg.delta_x:.3f}, DeltaY: {msg.delta_y:.3f}, 检测状态: {msg.detected}")
        else:
            # 如果没有找到任何可发布的目标
            self.currently_published_target_id = None
            # 注意：此处不能清除 last_known_target_id，因为我们需要记住它，以防它只是暂时消失。

    def update_tracks(self, current_detections):
        """
        根据新接收到的检测数据更新所有追踪目标的状态。
        """
        now = self.get_clock().now()
        for detection in current_detections:
            obj_id = detection.get('id')
            if obj_id is None: continue # 如果检测结果没有ID，则跳过
            
            u = detection.get('pixel_u', 0.0)
            v = detection.get('pixel_v', 0.0)

            # 如果是新目标，则初始化它的追踪信息
            if obj_id not in self.tracked_objects:
                initial_color = detection.get('class_name', '0') # 默认为'0'
                self.tracked_objects[obj_id] = {
                    'filtered_u': u, 
                    'filtered_v': v, 
                    'frame_window': [], 
                    'last_seen': now, 
                    'color': initial_color
                }
            
            track = self.tracked_objects[obj_id]
            track['last_seen'] = now # 更新最后见到时间
            
            # 使用指数移动平均(EMA)来平滑坐标，减少抖动
            track['filtered_u'] = self.alpha * u + (1.0 - self.alpha) * track['filtered_u']
            track['filtered_v'] = self.alpha * v + (1.0 - self.alpha) * track['filtered_v']
            
            # --- 颜色滤波逻辑 ---
            # 1. 更新颜色的滑动窗口
            track['frame_window'].append(detection)
            if len(track['frame_window']) > self.color_filter_window_size:
                track['frame_window'].pop(0)

            # 2. 从窗口中提取所有颜色
            colors_in_window = [d.get('class_name', '0') for d in track['frame_window']]
            
            # 3. 找到出现次数最多的颜色（计算众数）作为最终颜色
            if colors_in_window:
                most_common_color = max(set(colors_in_window), key=colors_in_window.count)
                track['color'] = most_common_color

    def arrived_callback(self, msg):
        """
        订阅 /processed_status 的回调函数。
        当机器人成功处理一个目标后，会收到一个True的消息。
        """
        if msg.data is True and self.currently_published_target_id is not None:
            target_id = self.currently_published_target_id
            self.get_logger().info(f"目标 {target_id} 已处理完成，将忽略该目标。")
            self.completed_target_ids.add(target_id)
            
            # 新增：如果完成的目标是我们锁定的目标，则释放锁定，以便可以选择新目标
            if target_id == self.last_known_target_id:
                self.last_known_target_id = None

            self.currently_published_target_id = None

    def cleanup_stale_tracks(self):
        """
        清理那些长时间没有被再次检测到的“过时”目标。
        """
        now = self.get_clock().now()
        # 找出所有超时的目标ID
        stale_ids = [obj_id for obj_id, track in self.tracked_objects.items() if (now - track['last_seen']).nanoseconds / 1e9 > self.track_timeout]
        
        for obj_id in stale_ids:
            if obj_id in self.tracked_objects:
                del self.tracked_objects[obj_id]
                self.get_logger().info(f"移除过时目标: ID {obj_id}")
            # 注意：此处不应从 completed_target_ids 中移除，因为一个目标完成了就是完成了。

    def client_loop(self, cli):
        """
        处理单个客户端连接的循环。
        """
        buf = ''
        try:
            while rclpy.ok():
                data = cli.recv(1024).decode('utf-8')
                if not data: break # 客户端关闭了连接
                buf += data
                # 因为数据是流式的，可能一次接收到多个或不完整的JSON对象，所以按换行符分割处理
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    if line.strip():
                        try:
                            frame_data = json.loads(line.strip())
                            # 调用核心的追踪更新函数
                            self.update_tracks(frame_data.get('objects', []))
                            # 在每一帧数据更新后，立即调用选择和发布逻辑
                            self.select_and_publish_target()
                        except json.JSONDecodeError:
                            self.get_logger().warn(f'收到无效的JSON数据: {line}')
        finally:
            # 当客户端断开连接时，重置所有状态，确保系统干净
            self.get_logger().info(f'客户端 {cli.getpeername()} 已断开。重置所有状态。')
            cli.close()
            self.tracked_objects.clear()
            self.completed_target_ids.clear()
            self.currently_published_target_id = None
            # 新增：在客户端断开时也重置锁定的目标ID
            self.last_known_target_id = None

    def is_stably_detected(self, track):
        """
        判断一个目标是否被“稳定”检测到。
        """
        # 使用新的检测阈值（值为1），只要检测到就认为是稳定的
        return len(track['frame_window']) >= self.required_frames_for_detection
        
    def accept_loop(self):
        """
        Socket服务器的主循环，用于接收新的客户端连接。
        """
        while rclpy.ok():
            try:
                cli, addr = self.sock.accept()
                self.get_logger().info(f'客户端 {addr} 已连接。')
                # 为每个客户端创建一个新的线程来处理数据
                threading.Thread(target=self.client_loop, args=(cli,), daemon=True).start()
            except Exception:
                pass # 在关闭socket时可能会有异常，忽略即可
            
    def destroy_node(self):
        """
        在节点销毁时，关闭socket。
        """
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()