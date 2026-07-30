#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msg_tool.msg import Detect
from geometry_msgs.msg import Point
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from .color_detector_core import ColorDetector, ColorDetectionResult
from .vision_task_manager import get_vision_task_manager, init_vision_task_manager
from typing import List, Tuple, Optional

# 定义检测结果消息类型
class RingDetectionMsg:
    def __init__(self):
        self.delta_x = 0.0  # depth (前进方向)
        self.delta_y = 0.0  # -x (左右方向，负号用于坐标系对齐)
        self.yaw = 0.0
        self.detected = False

class LowPassFilter:
    """低通滤波器类"""
    def __init__(self, alpha=0.7):
        self.alpha = alpha  # 滤波系数
        self.prev_delta_x = None
        self.prev_delta_y = None
        self.prev_yaw = None
        self.initialized = False
    
    def filter(self, delta_x, delta_y, yaw, detected):
        """对检测结果进行低通滤波"""
        if not detected:
            # 如果当前帧未检测到，返回上一次滤波后的值（如果存在）
            if self.initialized:
                return self.prev_delta_x, self.prev_delta_y, self.prev_yaw
            else:
                return delta_x, delta_y, yaw
        
        if not self.initialized:
            # 第一次检测到，直接使用当前值
            self.prev_delta_x = delta_x
            self.prev_delta_y = delta_y
            self.prev_yaw = yaw
            self.initialized = True
            return delta_x, delta_y, yaw
        
        # 应用低通滤波
        filtered_delta_x = self.alpha * self.prev_delta_x + (1 - self.alpha) * delta_x
        filtered_delta_y = self.alpha * self.prev_delta_y + (1 - self.alpha) * delta_y
        
        # 对角度进行特殊处理，考虑角度的周期性
        filtered_yaw = self._filter_angle(self.prev_yaw, yaw)
        
        # 更新历史值
        self.prev_delta_x = filtered_delta_x
        self.prev_delta_y = filtered_delta_y
        self.prev_yaw = filtered_yaw
        
        return filtered_delta_x, filtered_delta_y, filtered_yaw
    
    def _filter_angle(self, prev_angle, current_angle):
        """对角度进行滤波，考虑角度的周期性"""
        # 计算角度差，处理跨越±π的情况
        diff = current_angle - prev_angle
        
        # 将角度差限制在[-π, π]范围内
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        # 应用滤波
        filtered_angle = prev_angle + (1 - self.alpha) * diff
        
        # 将结果角度限制在[-π, π]范围内
        while filtered_angle > math.pi:
            filtered_angle -= 2 * math.pi
        while filtered_angle < -math.pi:
            filtered_angle += 2 * math.pi
        
        return filtered_angle
    
    def reset(self):
        """重置滤波器"""
        self.prev_delta_x = None
        self.prev_delta_y = None
        self.prev_yaw = None
        self.initialized = False

class RingDetector(Node):
    def __init__(self):
        super().__init__('ring_detector')

        # 初始化任务管理器
        # vision_task_subscriber_node = init_vision_task_manager()
        # 获取任务管理器
        self.vision_task_manager = get_vision_task_manager()
        
        # 初始化参数
        self.ring_diameter_m = 1.0
        self.ring_radius_m = self.ring_diameter_m / 2.0
        
        # 圆环检测的距离约束参数
        self.min_ring_distance = 0.8   # 最小圆环距离 (80cm)
        self.max_ring_distance = 1.2   # 最大圆环距离 (120cm)
        self.distance_tolerance = 0.2  # 距离容差 (±20cm)
        
        # 水平对齐约束参数
        self.horizontal_tolerance = 0.1  # 水平对齐容差 (±10cm)
        
        # 垂直线检测参数
        self.min_line_length = 40
        self.max_line_gap = 20
        self.vertical_angle_threshold = 10  # 与垂直方向的最大角度差
        
        # 深度图像处理参数
        self.depth_scale = 0.001  # 深度图像比例因子 (通常是毫米到米的转换)
        self.max_depth = 3.0  # 最大有效深度(米)
        self.min_depth = 0.1  # 最小有效深度(米)
        
        # 初始化低通滤波器
        self.filter_alpha = 0.7
        self.low_pass_filter = LowPassFilter(alpha=self.filter_alpha)
        
        # 滤波器重置计数器（用于处理长时间未检测到的情况）
        self.no_detection_count = 0
        self.max_no_detection_count = 10  # 1秒未检测到就重置滤波器（10Hz * 1s）
        
        # 初始化颜色检测器
        self.color_detector = ColorDetector(
            target_width=640,
            target_height=480,
            area_threshold=1000,  # 降低阈值以检测更小的区域
            coordinate_scale=1.5,
            open_iterations=2,
            close_iterations=3
        )
        
        # 相机内参（从颜色检测器获取）
        self.camera_matrix = self.color_detector.camera_matrix
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        self.fx, self.fy = fx, fy
        self.cx, self.cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        
        # 创建CV Bridge
        self.bridge = CvBridge()
        
        # 存储最新的图像
        self.latest_color_image = None
        self.latest_depth_image = None
        
        # 存储调试信息
        self.latest_detection_result = None
        self.latest_debug_data = None
        
        # 创建订阅者
        self.depth_subscriber = self.create_subscription(
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw', 
            self.depth_callback, 
            10)
        
        self.color_subscriber = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw',
            self.color_callback, 
            10)
        
        # 创建发布者
        self.result_publisher = self.create_publisher(
            Detect,
            '/ring_detection',
            10)
        
        # 创建定时器进行检测
        self.timer = self.create_timer(0.1, self.detection_timer_callback)  # 10Hz
        
        # 创建OpenCV窗口
        cv2.namedWindow('Ring Detection Debug', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Edge Detection', cv2.WINDOW_AUTOSIZE)

        self.get_logger().info('红色圆环检测器已初始化 (任务1)')
        self.get_logger().info(f'Ring diameter constraint: {self.min_ring_distance:.2f}m - {self.max_ring_distance:.2f}m')
        self.get_logger().info(f'Horizontal alignment tolerance: ±{self.horizontal_tolerance:.2f}m')
        self.get_logger().info(f'Low Pass Filter (alpha={self.filter_alpha})')

    def __del__(self):
        """析构函数，关闭OpenCV窗口"""
        cv2.destroyAllWindows()
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
    
    def color_callback(self, msg):
        """彩色图像回调"""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')
    
    def detection_timer_callback(self):
        """定时检测回调"""
        
        # 检查当前是否应该执行任务1
        if not self.vision_task_manager.is_task_active(1):
            # 如果不是任务1，跳过处理但保持节点运行
            return
        
        # self.get_logger().info('start ring detection (任务1)')
        
        if self.latest_color_image is None or self.latest_depth_image is None:
            return
        
        # 执行圆环检测
        result, debug_data = self.detect_ring(self.latest_color_image, self.latest_depth_image)
        
        # 应用低通滤波器
        filtered_result = self.apply_filter(result)
        
        # 发布滤波后的结果
        self.publish_result(filtered_result)
        
        # 显示调试图像（显示原始检测结果和滤波后结果）
        self.display_debug_images(self.latest_color_image, self.latest_depth_image, result, filtered_result, debug_data)
        
        # 检查退出键
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit key pressed, shutting down...')
            rclpy.shutdown()
    
    def apply_filter(self, result: RingDetectionMsg) -> RingDetectionMsg:
        """应用低通滤波器"""
        filtered_result = RingDetectionMsg()
        
        if result.detected:
            # 检测到目标，重置未检测计数器
            self.no_detection_count = 0
            
            # 应用滤波
            filtered_delta_x, filtered_delta_y, filtered_yaw = self.low_pass_filter.filter(
                result.delta_x, result.delta_y, result.yaw, result.detected
            )
            
            filtered_result.detected = True
            filtered_result.delta_x = filtered_delta_x
            filtered_result.delta_y = filtered_delta_y
            filtered_result.yaw = filtered_yaw
            
        else:
            # 未检测到目标
            self.no_detection_count += 1
            
            if self.no_detection_count > self.max_no_detection_count:
                # 长时间未检测到，重置滤波器
                self.low_pass_filter.reset()
                self.no_detection_count = 0
                self.get_logger().debug('Low pass filter reset due to long period without detection')
            
            # 尝试使用滤波器的上一次值
            if self.low_pass_filter.initialized:
                filtered_delta_x, filtered_delta_y, filtered_yaw = self.low_pass_filter.filter(
                    0.0, 0.0, 0.0, False
                )
                filtered_result.detected = False  # 标记为未检测到，但保留滤波后的位置估计
                filtered_result.delta_x = filtered_delta_x
                filtered_result.delta_y = filtered_delta_y
                filtered_result.yaw = filtered_yaw
            else:
                # 滤波器未初始化，使用原始结果
                filtered_result = result
        
        return filtered_result
    
    def get_depth_at_line(self, depth_image: np.ndarray, line: dict, num_samples: int = 10) -> Optional[float]:
        """沿着垂直线采样获取深度值"""
        x = int(line['x'])
        y_min = int(line['y_min'])
        y_max = int(line['y_max'])
        
        height, width = depth_image.shape
        
        # 确保坐标在图像范围内
        x = max(0, min(width - 1, x))
        y_min = max(0, min(height - 1, y_min))
        y_max = max(0, min(height - 1, y_max))
        
        if y_max <= y_min:
            return None
        
        # 沿着线段采样深度值
        valid_depths = []
        for i in range(num_samples):
            y = int(y_min + (y_max - y_min) * i / (num_samples - 1))
            
            # 在该点周围小区域采样
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    px, py = x + dx, y + dy
                    if 0 <= px < width and 0 <= py < height:
                        depth_value = depth_image[py, px] * self.depth_scale
                        if self.min_depth <= depth_value <= self.max_depth:
                            valid_depths.append(depth_value)
        
        if len(valid_depths) == 0:
            return None
        
        # 返回中位数深度值（更稳定）
        return np.median(valid_depths)
    
    def calculate_3d_position(self, x_2d: float, y_2d: float, depth: float) -> Tuple[float, float, float]:
        """将2D像素坐标和深度转换为3D坐标"""
        x_3d = (x_2d - self.cx) * depth / self.fx
        y_3d = (y_2d - self.cy) * depth / self.fy
        z_3d = depth
        return x_3d, y_3d, z_3d
    
    def validate_ring_geometry(self, left_3d: Tuple[float, float, float], 
                              right_3d: Tuple[float, float, float]) -> Tuple[bool, str, float]:
        """验证检测到的几何体是否符合圆环特征"""
        
        # 计算两点之间的3D距离
        dx = right_3d[0] - left_3d[0]
        dy = right_3d[1] - left_3d[1]
        dz = right_3d[2] - left_3d[2]
        distance_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # 检查: 3D距离是否接近圆环直径
        distance_error = abs(distance_3d - self.ring_diameter_m)
        if distance_error > self.distance_tolerance:
            return False, f"Distance error too large: {distance_error:.3f}m (tolerance: {self.distance_tolerance:.3f}m)", distance_3d
        
        return True, "Valid ring geometry", distance_3d
    
    def detect_ring(self, image: np.ndarray, depth_image: np.ndarray) -> Tuple[RingDetectionMsg, dict]:
        """检测红色圆环"""
        result = RingDetectionMsg()
        debug_data = {
            'red_mask': None,
            'edges': None,
            'vertical_lines': [],
            'left_line': None,
            'right_line': None,
            'left_3d': None,
            'right_3d': None,
            'ring_pose': None,
            'depth_info': None,
            'validation_msg': "No validation performed",
            'line_pairs_tested': 0,
            'best_pair_distance': None,
            'horizontal_deviation': None
        }
        
        try:
            # 使用颜色检测器检测红色区域
            red_result = self.color_detector.detect(image, colors='red')
            
            if not red_result.detected:
                debug_data['validation_msg'] = "No red color detected"
                return result, debug_data
            
            # 获取红色掩膜进行边缘检测
            processed_img = self.color_detector._crop_and_resize_image(image)
            processed_depth = self.color_detector._crop_and_resize_image(depth_image)
            hsv_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2HSV)
            
            # 创建红色掩膜
            red_config = self.color_detector.color_thresholds['red']
            mask = self.color_detector._create_mask(
                hsv_img, 
                red_config['ranges'], 
                self.color_detector.close_iterations,
                self.color_detector.open_iterations
            )
            debug_data['red_mask'] = mask
            
            # 边缘检测
            edges = cv2.Canny(mask, 50, 150, apertureSize=3)
            debug_data['edges'] = edges
            
            # 检测垂直直线
            vertical_lines = self.detect_vertical_lines(edges)
            debug_data['vertical_lines'] = vertical_lines
            
            if len(vertical_lines) < 2:
                debug_data['validation_msg'] = f"Insufficient vertical lines: {len(vertical_lines)} (need ≥2)"
                return result, debug_data
            
            # 基于3D距离和水平对齐选择圆环的左右两侧竖直线
            left_line, right_line, pair_info = self.select_ring_edges_by_3d_distance_and_alignment(vertical_lines, processed_depth)
            debug_data['left_line'] = left_line
            debug_data['right_line'] = right_line
            debug_data['line_pairs_tested'] = pair_info['pairs_tested']
            debug_data['best_pair_distance'] = pair_info['best_distance']
            debug_data['horizontal_deviation'] = pair_info['horizontal_deviation']
            
            if left_line is None or right_line is None:
                debug_data['validation_msg'] = f"No valid line pair found (tested {pair_info['pairs_tested']} pairs)"
                return result, debug_data
            
            # 获取左右两侧的深度值
            left_depth = self.get_depth_at_line(processed_depth, left_line)
            right_depth = self.get_depth_at_line(processed_depth, right_line)
            
            if left_depth is None or right_depth is None:
                debug_data['validation_msg'] = "Invalid depth values at selected edges"
                self.get_logger().warn("无法获取选中边缘的有效深度值")
                return result, debug_data
            
            # 计算左右两侧的3D坐标
            left_3d = self.calculate_3d_position(left_line['x'], left_line['center_y'], left_depth)
            right_3d = self.calculate_3d_position(right_line['x'], right_line['center_y'], right_depth)
            debug_data['left_3d'] = left_3d
            debug_data['right_3d'] = right_3d
            
            # 验证几何体是否符合圆环特征
            is_valid, validation_msg, distance_3d = self.validate_ring_geometry(left_3d, right_3d)
            debug_data['validation_msg'] = validation_msg
            
            if not is_valid:
                self.get_logger().debug(f"Ring validation failed: {validation_msg}")
                return result, debug_data
            
            # 基于两侧3D点计算圆环中心和yaw角度
            ring_pose = self.calculate_ring_pose_from_edges(left_3d, right_3d, left_line, right_line)
            debug_data['ring_pose'] = ring_pose
            
            if ring_pose is not None:
                result.detected = True
                # 按照机身坐标系对齐：delta_x=depth, delta_y=-x
                result.delta_x = ring_pose['center_3d'][2]  # 深度作为前进方向
                result.delta_y = -ring_pose['center_3d'][0]  # 左右偏移，负号用于坐标系对齐
                result.yaw = math.radians(ring_pose['yaw_angle'])  # 转换为弧度
                
                debug_data['depth_info'] = {
                    'left_depth': left_depth,
                    'right_depth': right_depth,
                    'center_depth': ring_pose['center_3d'][2],
                    'center_x': ring_pose['center_3d'][0],
                    'center_y': ring_pose['center_3d'][1]
                }
                
                self.get_logger().debug(
                    f'Ring detected: depth={result.delta_x:.3f}m, '
                    f'lateral_offset={result.delta_y:.3f}m, yaw={math.degrees(result.yaw):.1f}°, '
                    f'3D_distance={distance_3d:.3f}m, h_dev={debug_data["horizontal_deviation"]:.3f}m '
                    f'(tested {pair_info["pairs_tested"]} pairs)'
                )
            else:
                debug_data['validation_msg'] += " (pose calculation failed)"
        
        except Exception as e:
            self.get_logger().error(f'Error in ring detection: {e}')
            debug_data['validation_msg'] = f"Error: {str(e)}"
        
        return result, debug_data
    
    def select_ring_edges_by_3d_distance_and_alignment(self, vertical_lines: List[dict], depth_image: np.ndarray) -> Tuple[Optional[dict], Optional[dict], dict]:
        """基于3D距离和水平对齐选择圆环的左右两侧竖直线"""
        if len(vertical_lines) < 2:
            return None, None, {'pairs_tested': 0, 'best_distance': None, 'horizontal_deviation': None}
        
        # 为每条线计算代表性深度值
        lines_with_depth = []
        for line in vertical_lines:
            # 创建临时的线字典用于深度计算
            temp_line = {
                'x': line['center_x'],
                'y_min': min(line['points'][1], line['points'][3]),
                'y_max': max(line['points'][1], line['points'][3]),
                'center_y': line['center_y']
            }
            depth = self.get_depth_at_line(depth_image, temp_line)
            if depth is not None:
                lines_with_depth.append({
                    'line': line,
                    'depth': depth,
                    'temp_line': temp_line
                })
        
        if len(lines_with_depth) < 2:
            return None, None, {'pairs_tested': 0, 'best_distance': None, 'horizontal_deviation': None}
        
        # 尝试所有可能的线对组合
        best_pair = None
        best_distance = None
        best_horizontal_deviation = None
        min_combined_error = float('inf')
        pairs_tested = 0
        
        for i in range(len(lines_with_depth)):
            for j in range(i + 1, len(lines_with_depth)):
                pairs_tested += 1
                
                line1 = lines_with_depth[i]
                line2 = lines_with_depth[j]
                
                # 确保line1在左侧，line2在右侧
                if line1['line']['center_x'] > line2['line']['center_x']:
                    line1, line2 = line2, line1
                
                # 计算两条线对应点的3D坐标
                left_3d = self.calculate_3d_position(
                    line1['line']['center_x'], 
                    line1['line']['center_y'], 
                    line1['depth']
                )
                right_3d = self.calculate_3d_position(
                    line2['line']['center_x'], 
                    line2['line']['center_y'], 
                    line2['depth']
                )
                
                # 计算3D距离
                dx = right_3d[0] - left_3d[0]
                dy = right_3d[1] - left_3d[1]
                dz = right_3d[2] - left_3d[2]
                distance_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # 计算水平偏差（3D y坐标差）
                horizontal_deviation = abs(right_3d[1] - left_3d[1])
                
                # 计算与目标直径的误差
                distance_error = abs(distance_3d - self.ring_diameter_m)
                
                # 检查基本约束
                distance_valid = distance_error <= self.distance_tolerance
                horizontal_valid = horizontal_deviation <= self.horizontal_tolerance
                
                if distance_valid and horizontal_valid:
                    # 计算组合误差（距离误差 + 水平偏差的权重组合）
                    # 给水平对齐更高的权重，因为这是一个重要的几何约束
                    normalized_distance_error = distance_error / self.distance_tolerance
                    normalized_horizontal_error = horizontal_deviation / self.horizontal_tolerance
                    combined_error = 0.6 * normalized_distance_error + 0.4 * normalized_horizontal_error
                    
                    if combined_error < min_combined_error:
                        min_combined_error = combined_error
                        best_distance = distance_3d
                        best_horizontal_deviation = horizontal_deviation
                        best_pair = (line1, line2)
                        
                        self.get_logger().debug(
                            f'Better pair found: 3D distance={distance_3d:.3f}m, '
                            f'dist_error={distance_error:.3f}m, h_dev={horizontal_deviation:.3f}m, '
                            f'combined_error={combined_error:.3f}, x1={line1["line"]["center_x"]:.0f}, '
                            f'x2={line2["line"]["center_x"]:.0f}'
                        )
                else:
                    # 记录不满足约束的情况
                    self.get_logger().debug(
                        f'Pair rejected: dist_valid={distance_valid}({distance_error:.3f}m), '
                        f'h_valid={horizontal_valid}({horizontal_deviation:.3f}m), '
                        f'x1={line1["line"]["center_x"]:.0f}, x2={line2["line"]["center_x"]:.0f}'
                    )
        
        # 构建结果
        info = {
            'pairs_tested': pairs_tested,
            'best_distance': best_distance,
            'horizontal_deviation': best_horizontal_deviation
        }
        
        if best_pair is None:
            return None, None, info
        
        # 将选中的线转换为标准格式
        left_line_data = best_pair[0]
        right_line_data = best_pair[1]
        
        left_line = self.convert_to_fitted_line_format(left_line_data['line'])
        right_line = self.convert_to_fitted_line_format(right_line_data['line'])
        
        self.get_logger().debug(
            f'Selected ring edges: left_x={left_line["x"]:.1f}, right_x={right_line["x"]:.1f}, '
            f'3D_distance={best_distance:.3f}m, h_deviation={best_horizontal_deviation:.3f}m, '
            f'dist_error={abs(best_distance - self.ring_diameter_m):.3f}m'
        )
        
        return left_line, right_line, info
    
    def convert_to_fitted_line_format(self, line: dict) -> dict:
        """将检测到的线格式转换为拟合线格式"""
        x1, y1, x2, y2 = line['points']
        return {
            'x': line['center_x'],
            'y_min': min(y1, y2),
            'y_max': max(y1, y2),
            'center_y': line['center_y'],
            'length': line['length'],
            'confidence': 1  # 单条线的置信度为1
        }
    
    def calculate_ring_pose_from_edges(self, left_3d: Tuple[float, float, float], 
                                     right_3d: Tuple[float, float, float],
                                     left_line: dict, right_line: dict) -> Optional[dict]:
        """基于两侧边缘的3D坐标计算圆环中心和yaw角度"""
        
        # 计算两点之间的3D距离
        dx = right_3d[0] - left_3d[0]
        dy = right_3d[1] - left_3d[1]
        dz = right_3d[2] - left_3d[2]
        distance_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
        yaw_flag = 1
        
        # 计算圆环中心的3D坐标（两侧中点）
        center_3d = (
            (left_3d[0] + right_3d[0]) / 2,
            (left_3d[1] + right_3d[1]) / 2,
            (left_3d[2] + right_3d[2]) / 2
        )

        if dz < 0:
            yaw_flag = -1
        else:
            yaw_flag = 1
        
        # 计算yaw角度
        # 使用更准确的yaw计算：基于X方向的投影
        cos_yaw = dx / self.ring_diameter_m
        
        # 限制cos_yaw在有效范围内
        cos_yaw = np.clip(cos_yaw, -1, 1)
        yaw_angle = yaw_flag * math.degrees(math.acos(abs(cos_yaw)))
        
        # 计算圆环在图像中的2D中心
        center_2d = (
            (left_line['x'] + right_line['x']) / 2,
            (left_line['center_y'] + right_line['center_y']) / 2
        )
        
        # 计算像素距离用于调试
        pixel_distance = abs(right_line['x'] - left_line['x'])
        
        return {
            'center_2d': center_2d,
            'center_3d': center_3d,
            'left_3d': left_3d,
            'right_3d': right_3d,
            'distance_3d': distance_3d,
            'yaw_angle': yaw_angle,
            'pixel_width': pixel_distance,
            'cos_yaw': cos_yaw
        }
    
    def detect_vertical_lines(self, edges: np.ndarray) -> List[dict]:
        """检测竖直直线"""
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        vertical_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # 计算直线角度
                if abs(x2 - x1) < 1:
                    angle = 90
                else:
                    angle = abs(math.degrees(math.atan((y2 - y1) / (x2 - x1))))
                
                # 筛选接近垂直的直线
                if angle > 90 - self.vertical_angle_threshold:
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    
                    vertical_lines.append({
                        'points': (x1, y1, x2, y2),
                        'center_x': center_x,
                        'center_y': center_y,
                        'length': length,
                        'angle': angle
                    })
        
        return vertical_lines
    
    def cluster_vertical_lines(self, vertical_lines: List[dict]) -> Tuple[List[dict], List[dict]]:
        """将垂直直线聚类为左右两组（保留原方法作为备用）"""
        if len(vertical_lines) < 2:
            return [], []
        
        # 按x坐标排序
        vertical_lines.sort(key=lambda x: x['center_x'])
        
        # 找到最大间隙进行分组
        if len(vertical_lines) >= 4:
            x_coords = [line['center_x'] for line in vertical_lines]
            gaps = []
            for i in range(1, len(x_coords)):
                gaps.append((x_coords[i] - x_coords[i-1], i))
            
            # 找到最大的间隙
            max_gap, split_idx = max(gaps)
            if max_gap > 30:  # 降低间隙阈值
                left_group = vertical_lines[:split_idx]
                right_group = vertical_lines[split_idx:]
            else:
                # 简单分组：前一半为左组，后一半为右组
                mid_point = len(vertical_lines) // 2
                left_group = vertical_lines[:mid_point]
                right_group = vertical_lines[mid_point:]
        else:
            # 简单分组
            mid_point = len(vertical_lines) // 2
            left_group = vertical_lines[:mid_point]
            right_group = vertical_lines[mid_point:]
        
        return left_group, right_group
    
    def fit_vertical_line_to_group(self, line_group: List[dict]) -> Optional[dict]:
        """将一组直线拟合成一条垂直直线"""
        if not line_group:
            return None
        
        # 加权平均计算x坐标
        total_weight = sum(line['length'] for line in line_group)
        if total_weight == 0:
            return None
        
        weighted_x = sum(line['center_x'] * line['length'] for line in line_group) / total_weight
        
        # 计算y坐标范围
        all_y_coords = []
        for line in line_group:
            x1, y1, x2, y2 = line['points']
            all_y_coords.extend([y1, y2])
        
        min_y = min(all_y_coords)
        max_y = max(all_y_coords)
        center_y = (min_y + max_y) / 2
        
        return {
            'x': weighted_x,
            'y_min': min_y,
            'y_max': max_y,
            'center_y': center_y,
            'length': max_y - min_y,
            'confidence': len(line_group)
        }
    
    def publish_result(self, result: RingDetectionMsg):
        """发布检测结果"""
        msg = Detect()
        msg.delta_x = result.delta_x
        msg.delta_y = result.delta_y
        msg.yaw = result.yaw
        msg.detected = result.detected
        self.result_publisher.publish(msg)
    
    def display_debug_images(self, original_image: np.ndarray, depth_image: np.ndarray, 
                           original_result: RingDetectionMsg, filtered_result: RingDetectionMsg, debug_data: dict):
        """使用cv2.imshow显示调试图像"""
        try:
            # 获取处理后的图像
            processed_img = self.color_detector._crop_and_resize_image(original_image)
            
            if debug_data['edges'] is not None:
                edges_display = cv2.cvtColor(debug_data['edges'], cv2.COLOR_GRAY2BGR)
                cv2.imshow('Edge Detection', edges_display)

            debug_image = self.draw_detection_result(processed_img, original_result, filtered_result, debug_data)
            cv2.imshow('Ring Detection Debug', debug_image)
            
        except Exception as e:
            self.get_logger().error(f'Error displaying debug images: {e}')

    
    def draw_detection_result(self, image: np.ndarray, original_result: RingDetectionMsg, 
                            filtered_result: RingDetectionMsg, debug_data: dict) -> np.ndarray:
        """绘制检测结果（显示原始和滤波后的结果）"""
        debug_image = image.copy()
        height, width = debug_image.shape[:2]
        
        # 绘制图像中心十字
        center_x, center_y = width // 2, height // 2
        cv2.line(debug_image, (center_x - 20, center_y), (center_x + 20, center_y), (128, 128, 128), 1)
        cv2.line(debug_image, (center_x, center_y - 20), (center_x, center_y + 20), (128, 128, 128), 1)
        
        # 绘制检测到的垂直直线
        for i, line in enumerate(debug_data['vertical_lines']):
            x1, y1, x2, y2 = line['points']
            cv2.line(debug_image, (x1, y1), (x2, y2), (255, 255, 0), 1)  # 黄色原始线段
        
        # 绘制拟合后的左右直线
        if debug_data['left_line'] is not None:
            left_line = debug_data['left_line']
            cv2.line(debug_image, 
                    (int(left_line['x']), int(left_line['y_min'])),
                    (int(left_line['x']), int(left_line['y_max'])),
                    (0, 255, 0), 3)  # 绿色左侧线
            cv2.putText(debug_image, "L", 
                       (int(left_line['x']) - 20, int(left_line['center_y'])),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        if debug_data['right_line'] is not None:
            right_line = debug_data['right_line']
            cv2.line(debug_image,
                    (int(right_line['x']), int(right_line['y_min'])),
                    (int(right_line['x']), int(right_line['y_max'])),
                    (0, 255, 0), 3)  # 绿色右侧线
            cv2.putText(debug_image, "R", 
                       (int(right_line['x']) + 10, int(right_line['center_y'])),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # 如果检测成功，绘制更多信息
        if original_result.detected and debug_data['ring_pose'] is not None:
            ring_pose = debug_data['ring_pose']
            center_2d = ring_pose['center_2d']
            center_int = (int(center_2d[0]), int(center_2d[1]))
            
            # 绘制圆环中心点
            cv2.circle(debug_image, center_int, 8, (255, 0, 0), -1)  # 蓝色中心点
            
            # 绘制连接线
            if debug_data['left_line'] and debug_data['right_line']:
                cv2.line(debug_image,
                        (int(debug_data['left_line']['x']), int(debug_data['left_line']['center_y'])),
                        (int(debug_data['right_line']['x']), int(debug_data['right_line']['center_y'])),
                        (255, 0, 255), 2)  # 紫色连接线
            
            # 显示3D坐标信息
            if debug_data['left_3d'] and debug_data['right_3d']:
                left_3d = debug_data['left_3d']
                right_3d = debug_data['right_3d']
                
                # 在左右边缘显示深度信息
                cv2.putText(debug_image, f"{left_3d[2]:.2f}m", 
                           (int(debug_data['left_line']['x']) - 30, int(debug_data['left_line']['center_y']) + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(debug_image, f"{right_3d[2]:.2f}m", 
                           (int(debug_data['right_line']['x']) + 10, int(debug_data['right_line']['center_y']) + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # 显示检测信息（原始和滤波后的结果）
        info_texts = [
            f"Ring Detected: {original_result.detected}",
        ]
        
        # 添加验证信息
        validation_status = "PASS" if original_result.detected else "FAIL"
        validation_color = (0, 255, 0) if original_result.detected else (0, 0, 255)
        info_texts.append(f"Validation: {validation_status}")
        
        if original_result.detected:
            info_texts.extend([
                f"--- Original ---",
                f"Depth: {original_result.delta_x:.3f}m",
                f"Lateral: {-original_result.delta_y:.3f}m",
                f"Yaw: {math.degrees(original_result.yaw):.1f}°",
                f"--- Filtered (α={self.filter_alpha}) ---",
                f"Depth: {filtered_result.delta_x:.3f}m",
                f"Lateral: {-filtered_result.delta_y:.3f}m",
                f"Yaw: {math.degrees(filtered_result.yaw):.1f}°",
            ])
            
            # 如果有调试数据，添加3D距离信息
            if debug_data['ring_pose'] is not None:
                ring_pose = debug_data['ring_pose']
                distance_error = abs(ring_pose['distance_3d'] - self.ring_diameter_m)
                info_texts.extend([
                    f"3D Distance: {ring_pose['distance_3d']:.3f}m",
                    f"Distance Error: {distance_error:.3f}m",
                    f"Pixel Width: {ring_pose['pixel_width']:.0f}px"
                ])
                
            # 添加水平偏差信息
            if debug_data['horizontal_deviation'] is not None:
                h_dev = debug_data['horizontal_deviation']
                h_status = "OK" if h_dev <= self.horizontal_tolerance else "EXCEED"
                h_color = (0, 255, 0) if h_dev <= self.horizontal_tolerance else (0, 0, 255)
                info_texts.append(f"Horizontal Dev: {h_dev:.3f}m ({h_status})")
        
        # 显示线对测试信息
        if 'line_pairs_tested' in debug_data:
            info_texts.append(f"Line Pairs Tested: {debug_data['line_pairs_tested']}")
            if debug_data['best_pair_distance'] is not None:
                info_texts.append(f"Best Pair Distance: {debug_data['best_pair_distance']:.3f}m")
        
        # 显示验证信息
        info_texts.append(f"Validation: {debug_data['validation_msg']}")
        
        # 添加滤波器状态信息
        if self.low_pass_filter.initialized:
            info_texts.append(f"Filter: Initialized")
        else:
            info_texts.append(f"Filter: Not Initialized")
        
        info_texts.append(f"No Detection Count: {self.no_detection_count}")
        
        for i, text in enumerate(info_texts):
            if "Original" in text or "Filtered" in text:
                color = (0, 255, 255)  # 黄色标题
            elif i == 0:
                color = (0, 255, 0) if original_result.detected else (0, 0, 255)
            elif i == 1:  # Validation status
                color = validation_color
            elif "Filter:" in text:
                color = (255, 0, 255)  # 紫色滤波器状态
            elif "Validation:" in text and i > 1:
                color = (0, 255, 255)  # 青色验证详情
            elif "Distance Error:" in text:
                color = (0, 255, 0) if "0.0" in text else (255, 255, 0)  # 绿色或黄色
            elif "Horizontal Dev:" in text:
                h_dev = debug_data['horizontal_deviation']
                color = (0, 255, 0) if h_dev and h_dev <= self.horizontal_tolerance else (0, 0, 255)
            elif "Line Pairs Tested:" in text or "Best Pair Distance:" in text:
                color = (255, 255, 0)  # 黄色线对信息
            else:
                color = (255, 255, 255)  # 白色
            
            cv2.putText(debug_image, text,
                       (10, 20 + i * 16),
                       cv2.FONT_HERSHEY_SIMPLEX,
                       0.4, color, 1)
        
        # 在右下角显示约束参数
        constraint_info = [
            f"Ring Diameter: {self.ring_diameter_m:.1f}m",
            f"Distance Range: {self.min_ring_distance:.1f}-{self.max_ring_distance:.1f}m",
            f"Distance Tolerance: ±{self.distance_tolerance:.2f}m",
            f"Horizontal Tolerance: ±{self.horizontal_tolerance:.2f}m",
            f"Filter α = {self.filter_alpha}",
        ]
        
        for i, text in enumerate(constraint_info):
            cv2.putText(debug_image, text,
                       (width - 320, height - 75 + i * 15),
                       cv2.FONT_HERSHEY_SIMPLEX,
                       0.4, (200, 200, 200), 1)
        
        return debug_image

def main(args=None):
    rclpy.init(args=args)
    
    try:
        task_subscriber = init_vision_task_manager()
        # 创建圆环检测器
        detector = RingDetector()
        
        # 创建执行器并添加两个节点
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(task_subscriber)
        executor.add_node(detector)
        
        # 运行
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()