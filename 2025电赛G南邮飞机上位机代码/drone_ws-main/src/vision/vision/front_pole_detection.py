#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from msg_tool.msg import Color
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import traceback
from .vision_task_manager import get_vision_task_manager, init_vision_task_manager
from pyzbar import pyzbar
from std_msgs.msg import Int32
from collections import deque

class DepthBasedPoleDetector(Node):
    def __init__(self):
        super().__init__('depth_based_pole_detector')
        
        # 获取任务管理器
        self.vision_task_manager = get_vision_task_manager()
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 初始化图像存储
        self.depth_image = None
        self.color_image = None
        
        # 深度图像累计参数
        self.depth_frame_count = 6  # 累计帧数
        self.depth_frame_buffer = deque(maxlen=self.depth_frame_count)  # 深度图像缓冲区
        self.accumulated_depth = None  # 累计后的深度图像
        
        # 深度检测参数
        self.min_depth = 0.1
        self.max_depth = 2.0
        
        # 线段检测参数 - 针对高杆的两条边缘线
        self.hough_threshold = 50      # 霍夫变换阈值
        self.min_line_length = 100     # 最小线段长度
        self.max_line_gap = 20         # 最大线段间隙
        self.min_pole_width = 10       # 最小杆宽度（两条线之间的距离）
        self.max_pole_width = 100      # 最大杆宽度
        self.line_angle_tolerance = 15 # 线段角度容差（度）
        self.vertical_angle_threshold = 20  # 垂直度阈值（与垂直线的最大角度差）
        
        # 边缘检测后的轮廓滤除参数（保留用于处理完整轮廓）
        self.min_contour_perimeter_after_edge = 1000  # 降低要求
        
        # 杆检测参数（保留用于处理完整轮廓）
        self.min_pole_perimeter = 2000    # 降低要求
        self.aspect_ratio_threshold = 1.5
        
        # 形态学操作参数
        self.open_kernel_size = 3
        self.close_kernel_size = 3
        self.open_iterations = 2
        self.close_iterations = 3
        
        # Canny边缘检测参数
        self.canny_low_threshold = 50
        self.canny_high_threshold = 150
        self.gaussian_blur_kernel = 5
        
        # 条形码扫描相关参数
        self.barcode_scan_enabled = True
        self.zoom_factor = 2.0
        self.published_barcodes = set()

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
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
        self.detections_publisher = self.create_publisher(
            Color,
            '/detected_pole',
            10)
        
        self.barcode_publisher = self.create_publisher(
            Int32,
            '/flash_id',
            qos_profile)
        
        # 创建定时器用于图像处理
        self.timer = self.create_timer(0.1, self.process_images)
        
        # 相机内参
        self.camera_matrix = np.array([
            [908.4974365234375, 0, 649.4335327148438],
            [0, 906.1259765625, 360.55584716796875],
            [0, 0, 1]
        ])

        # 添加标记表明已收到图像
        self.got_depth = False
        self.got_color = False
        
        # 初始化低通滤波器
        self.init_low_pass_filter()
        
        self.get_logger().info(f'基于深度的杆检测器已初始化 (任务2) - 支持线段检测高杆')

    def init_low_pass_filter(self):
        """初始化低通滤波器"""
        self.alpha = 0.7
        self.filter_state = {
            'filtered_x': 0.0,
            'filtered_y': 0.0,
            'initialized': False
        }
    
    def apply_low_pass_filter(self, x, y):
        """应用低通滤波器"""
        if not self.filter_state['initialized']:
            self.filter_state['filtered_x'] = x
            self.filter_state['filtered_y'] = y
            self.filter_state['initialized'] = True
        else:
            self.filter_state['filtered_x'] = self.alpha * x + (1 - self.alpha) * self.filter_state['filtered_x']
            self.filter_state['filtered_y'] = self.alpha * y + (1 - self.alpha) * self.filter_state['filtered_y']
        
        return self.filter_state['filtered_x'], self.filter_state['filtered_y']
    
    def reset_filter(self):
        """重置滤波器"""
        self.filter_state['initialized'] = False
    
    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg)
            
            # 将深度图像添加到缓冲区
            self.depth_frame_buffer.append(depth_image.copy())
            
            # 累计深度图像
            self.accumulate_depth_frames()
            
            self.got_depth = True
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {str(e)}')
    
    def accumulate_depth_frames(self):
        """累计深度图像帧"""
        try:
            if len(self.depth_frame_buffer) < self.depth_frame_count:
                return
            
            depth_frames = []
            for depth_frame in self.depth_frame_buffer:
                depth_float = depth_frame.astype(np.float32)
                depth_frames.append(depth_float)
            
            depth_stack = np.stack(depth_frames, axis=-1)
            self.accumulated_depth = np.median(depth_stack, axis=-1)
            self.accumulated_depth = self.accumulated_depth.astype(self.depth_frame_buffer[0].dtype)
            
            self.get_logger().debug(f'累计了 {len(self.depth_frame_buffer)} 帧深度图像')
            
        except Exception as e:
            self.get_logger().error(f'累计深度图像时出错: {str(e)}')
    
    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.got_color = True
        except Exception as e:
            self.get_logger().error(f'处理彩色图像时出错: {str(e)}')

    def calculate_line_angle(self, x1, y1, x2, y2):
        """计算线段与垂直方向的角度差"""
        if x2 == x1:
            return 0  # 完全垂直
        
        # 计算线段角度（相对于水平方向）
        angle_rad = np.arctan2(y2 - y1, x2 - x1)
        angle_deg = np.degrees(angle_rad)
        
        # 转换为与垂直方向的角度差
        vertical_angle_diff = abs(abs(angle_deg) - 90)
        return min(vertical_angle_diff, abs(vertical_angle_diff - 180))

    def detect_pole_lines(self, edges_image):
        """检测杆的边缘线段"""
        try:
            # 使用霍夫变换检测线段
            lines = cv2.HoughLinesP(
                edges_image,
                rho=1,
                theta=np.pi/180,
                threshold=self.hough_threshold,
                minLineLength=self.min_line_length,
                maxLineGap=self.max_line_gap
            )
            
            if lines is None:
                return None
            
            # 筛选近似垂直的线段
            vertical_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle_diff = self.calculate_line_angle(x1, y1, x2, y2)
                
                if angle_diff <= self.vertical_angle_threshold:
                    line_info = {
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'length': np.sqrt((x2-x1)**2 + (y2-y1)**2),
                        'angle_diff': angle_diff,
                        'center_x': (x1 + x2) // 2,
                        'center_y': (y1 + y2) // 2
                    }
                    vertical_lines.append(line_info)
            
            if len(vertical_lines) < 2:
                return None
            
            # 寻找成对的线段（杆的两条边）
            pole_pairs = []
            for i, line1 in enumerate(vertical_lines):
                for j, line2 in enumerate(vertical_lines[i+1:], i+1):
                    # 计算两条线段的距离（取中心点距离）
                    center_dist = abs(line1['center_x'] - line2['center_x'])
                    
                    # 检查距离是否在杆宽度范围内
                    if self.min_pole_width <= center_dist <= self.max_pole_width:
                        # 检查两条线段的Y坐标重叠度
                        y1_min, y1_max = min(line1['y1'], line1['y2']), max(line1['y1'], line1['y2'])
                        y2_min, y2_max = min(line2['y1'], line2['y2']), max(line2['y1'], line2['y2'])
                        
                        overlap_start = max(y1_min, y2_min)
                        overlap_end = min(y1_max, y2_max)
                        overlap_length = max(0, overlap_end - overlap_start)
                        
                        # 要求有足够的Y坐标重叠
                        min_overlap = min(line1['length'], line2['length']) * 0.3
                        if overlap_length >= min_overlap:
                            pair_info = {
                                'line1': line1,
                                'line2': line2,
                                'width': center_dist,
                                'center_x': (line1['center_x'] + line2['center_x']) // 2,
                                'center_y': (overlap_start + overlap_end) // 2,
                                'overlap_length': overlap_length,
                                'avg_length': (line1['length'] + line2['length']) / 2
                            }
                            pole_pairs.append(pair_info)
            
            if not pole_pairs:
                return None
            
            # 选择最好的杆候选（重叠最长、线段最长）
            best_pair = max(pole_pairs, key=lambda p: p['overlap_length'] * p['avg_length'])
            
            return best_pair
            
        except Exception as e:
            self.get_logger().error(f'线段检测时出错: {str(e)}')
            return None

    def filter_short_contours(self, binary_image, min_perimeter):
        """滤除周长小于指定值的轮廓"""
        try:
            contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filtered_image = np.zeros_like(binary_image)
            
            original_contour_count = len(contours)
            filtered_contour_count = 0
            
            for contour in contours:
                perimeter = cv2.arcLength(contour, True)
                if perimeter >= min_perimeter:
                    cv2.fillPoly(filtered_image, [contour], 255)
                    filtered_contour_count += 1
            
            self.get_logger().debug(f'轮廓周长滤除: {original_contour_count} -> {filtered_contour_count}')
            
            return filtered_image, original_contour_count, filtered_contour_count
            
        except Exception as e:
            self.get_logger().error(f'滤除短轮廓时出错: {str(e)}')
            return binary_image, 0, 0

    def preprocess_depth_image(self, depth_image):
        """预处理深度图像"""
        try:
            # 将深度图像转换为毫米单位的浮点数
            depth_mm = depth_image.astype(np.float32)
            
            # 创建有效深度掩码
            valid_depth_mask = (depth_mm > self.min_depth * 1000) & (depth_mm < self.max_depth * 1000) & (depth_mm > 0)
            
            # 归一化处理
            depth_normalized = np.zeros_like(depth_mm, dtype=np.uint8)
            valid_pixels = valid_depth_mask
            if np.any(valid_pixels):
                min_valid_depth = np.min(depth_mm[valid_pixels])
                max_valid_depth = np.max(depth_mm[valid_pixels])
                if max_valid_depth > min_valid_depth:
                    depth_normalized[valid_pixels] = ((depth_mm[valid_pixels] - min_valid_depth) / 
                                                    (max_valid_depth - min_valid_depth) * 255).astype(np.uint8)
            
            depth_masked = np.where(valid_depth_mask, depth_normalized, 0).astype(np.uint8)
            
            # 高斯模糊
            if self.gaussian_blur_kernel > 0:
                depth_blurred = cv2.GaussianBlur(depth_masked, (self.gaussian_blur_kernel, self.gaussian_blur_kernel), 0)
            else:
                depth_blurred = depth_masked
            
            # 形态学操作
            open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.open_kernel_size, self.open_kernel_size))
            depth_opened = cv2.morphologyEx(depth_blurred, cv2.MORPH_OPEN, open_kernel, iterations=self.open_iterations)
            
            close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.close_kernel_size, self.close_kernel_size))
            depth_closed = cv2.morphologyEx(depth_opened, cv2.MORPH_CLOSE, close_kernel, iterations=self.close_iterations)
            
            # Canny边缘检测
            edges = cv2.Canny(depth_closed, self.canny_low_threshold, self.canny_high_threshold)
            
            # 边缘增强
            edge_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            edges_enhanced = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, edge_kernel, iterations=1)
            
            return edges_enhanced, depth_mm
            
        except Exception as e:
            self.get_logger().error(f'深度图像预处理时出错: {str(e)}')
            return None, None

    def detect_poles_from_depth(self, binary_image, depth_image):
        """从深度图像中检测杆状物体 - 支持线段检测和轮廓检测"""
        try:
            # 方法1: 线段检测（适用于高杆超出画面的情况）
            pole_lines = self.detect_pole_lines(binary_image)
            
            if pole_lines is not None:
                # 使用线段检测结果
                cx, cy = pole_lines['center_x'], pole_lines['center_y']
                
                # 获取深度信息
                if (0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]):
                    depth_mm = depth_image[cy, cx]
                    
                    if (depth_mm > self.min_depth * 1000 and 
                        depth_mm < self.max_depth * 1000 and 
                        depth_mm > 0):
                        
                        depth_m = float(depth_mm) / 1000.0
                        
                        pole_info = {
                            'center_x': cx,
                            'center_y': cy,
                            'depth_m': depth_m,
                            'width': pole_lines['width'],
                            'overlap_length': pole_lines['overlap_length'],
                            'detection_method': 'line_detection',
                            'line1': pole_lines['line1'],
                            'line2': pole_lines['line2']
                        }
                        
                        self.get_logger().debug(f'线段检测到杆: 宽度={pole_lines["width"]:.1f}px, 重叠长度={pole_lines["overlap_length"]:.1f}px, 深度={depth_m:.3f}m')
                        
                        # 绘制线段检测结果
                        self.draw_line_detection_result(binary_image, pole_lines)
                        
                        return pole_info
            
            # 方法2: 传统轮廓检测（适用于完整杆的情况）
            edges_filtered, _, _ = self.filter_short_contours(binary_image, self.min_contour_perimeter_after_edge)
            contours, _ = cv2.findContours(edges_filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            contour_display = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
            valid_poles = []
            
            for contour in contours:
                perimeter = cv2.arcLength(contour, True)
                if perimeter < self.min_pole_perimeter:
                    continue
                
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = h / w if w > 0 else 0
                if aspect_ratio < self.aspect_ratio_threshold:
                    continue
                
                center_x = x + w // 2
                center_y = y + h // 2
                
                if (0 <= center_y < depth_image.shape[0] and 
                    0 <= center_x < depth_image.shape[1]):
                    depth_mm = depth_image[center_y, center_x]
                    
                    if (depth_mm > self.min_depth * 1000 and 
                        depth_mm < self.max_depth * 1000 and 
                        depth_mm > 0):
                        
                        depth_m = float(depth_mm) / 1000.0
                        area = cv2.contourArea(contour)
                        
                        pole_info = {
                            'contour': contour,
                            'center_x': center_x,
                            'center_y': center_y,
                            'depth_m': depth_m,
                            'area': area,
                            'perimeter': perimeter,
                            'aspect_ratio': aspect_ratio,
                            'bbox': (x, y, w, h),
                            'detection_method': 'contour_detection'
                        }
                        
                        valid_poles.append(pole_info)
                        
                        # 绘制轮廓检测结果
                        cv2.drawContours(contour_display, [contour], -1, (0, 255, 0), 2)
                        cv2.rectangle(contour_display, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        cv2.circle(contour_display, (center_x, center_y), 5, (0, 0, 255), -1)
            
            cv2.imshow('Contour Detection', contour_display)
            
            if valid_poles:
                valid_poles.sort(key=lambda p: p['depth_m'])
                selected_pole = valid_poles[0]
                self.get_logger().info(f'轮廓检测到杆: 周长={selected_pole["perimeter"]:.1f}px, 深度={selected_pole["depth_m"]:.3f}m')
                return selected_pole
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'杆检测时出错: {str(e)}')
            return None

    def draw_line_detection_result(self, binary_image, pole_lines):
        """绘制线段检测结果"""
        try:
            # 创建彩色图像显示线段检测结果
            line_display = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
            
            line1 = pole_lines['line1']
            line2 = pole_lines['line2']
            
            # 绘制两条线段
            cv2.line(line_display, (line1['x1'], line1['y1']), (line1['x2'], line1['y2']), (0, 255, 0), 3)
            cv2.line(line_display, (line2['x1'], line2['y1']), (line2['x2'], line2['y2']), (0, 255, 0), 3)
            
            # 绘制中心点
            cv2.circle(line_display, (pole_lines['center_x'], pole_lines['center_y']), 8, (0, 0, 255), -1)
            
            # 绘制连接线显示宽度
            cv2.line(line_display, (line1['center_x'], line1['center_y']), 
                    (line2['center_x'], line2['center_y']), (255, 0, 0), 2)
            
            # 添加文本信息
            info_text = f"Width: {pole_lines['width']:.0f}px, Overlap: {pole_lines['overlap_length']:.0f}px"
            cv2.putText(line_display, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(line_display, "Line Detection Mode", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow('Line Detection', line_display)
            
        except Exception as e:
            self.get_logger().error(f'绘制线段检测结果时出错: {str(e)}')

    def crop_and_zoom_image(self, image, center_x, center_y, zoom_factor=2.0):
        """以指定中心点为中心，裁剪并放大图像"""
        try:
            h, w = image.shape[:2]
            crop_w = int(w / zoom_factor)
            crop_h = int(h / zoom_factor)
            
            x1 = max(0, center_x - crop_w // 2)
            y1 = max(0, center_y - crop_h // 2)
            x2 = min(w, x1 + crop_w)
            y2 = min(h, y1 + crop_h)
            
            if x2 - x1 < crop_w:
                if x1 == 0:
                    x2 = min(w, x1 + crop_w)
                else:
                    x1 = max(0, x2 - crop_w)
            
            if y2 - y1 < crop_h:
                if y1 == 0:
                    y2 = min(h, y1 + crop_h)
                else:
                    y1 = max(0, y2 - crop_h)
            
            cropped = image[y1:y2, x1:x2]
            zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_CUBIC)
            
            return zoomed, (x1, y1, x2, y2)
            
        except Exception as e:
            self.get_logger().error(f'图像裁剪放大时出错: {str(e)}')
            return image, None

    def preprocess_for_barcode(self, image):
        """条形码扫描预处理"""
        try:
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image.copy()
            return gray
        except Exception as e:
            self.get_logger().error(f'条形码预处理时出错: {str(e)}')
            return image

    def scan_barcode(self, image, center_x, center_y):
        """扫描条形码"""
        try:
            zoomed_image, crop_info = self.crop_and_zoom_image(image, center_x, center_y, self.zoom_factor)
            processed_image = self.preprocess_for_barcode(zoomed_image)
            barcodes = pyzbar.decode(processed_image)
            
            barcode_results = []
            for barcode in barcodes:
                barcode_data = barcode.data.decode('utf-8')
                barcode_type = barcode.type
                (x, y, w, h) = barcode.rect
                
                barcode_info = {
                    'data': barcode_data,
                    'type': barcode_type,
                    'position': (x, y, w, h),
                    'center_x': center_x,
                    'center_y': center_y
                }
                
                barcode_results.append(barcode_info)
                self.get_logger().info(f'扫描到条形码: {barcode_type} - {barcode_data}')
            
            return barcode_results
            
        except Exception as e:
            self.get_logger().error(f'条形码扫描时出错: {str(e)}')
            return []

    def publish_barcode_content(self, barcode_results):
        """发布条形码内容"""
        try:
            if not barcode_results:
                return
            
            for barcode in barcode_results:
                content = barcode['data']
                
                if content.isdigit() and len(content) == 4:
                    barcode_number = int(content)
                    
                    if barcode_number not in self.published_barcodes:
                        self.published_barcodes.add(barcode_number)
                        
                        msg = Int32()
                        msg.data = barcode_number
                        self.barcode_publisher.publish(msg)
                        
                        self.get_logger().info(f'发布新条形码内容: {barcode_number}')
                else:
                    self.get_logger().warn(f'条形码内容不是4位数字: {content}')
                
        except Exception as e:
            self.get_logger().error(f'发布条形码内容时出错: {str(e)}')

    def process_images(self):
        """图像处理定时器回调"""
        
        if not self.vision_task_manager.is_task_active(1):
            return
        
        if not self.got_depth or not self.got_color:
            return
        
        if len(self.depth_frame_buffer) < self.depth_frame_count:
            self.get_logger().debug(f'等待更多深度帧: {len(self.depth_frame_buffer)}/{self.depth_frame_count}')
            return
            
        if self.accumulated_depth is None or self.color_image is None:
            return
        
        if self.accumulated_depth.size == 0 or self.color_image.size == 0:
            return
        
        try:
            color_image = self.color_image.copy()
            depth_image = self.accumulated_depth.copy()
            
            binary_image, depth_mm = self.preprocess_depth_image(depth_image)
            
            if binary_image is None or depth_mm is None:
                self.publish_empty_detection()
                return
            
            pole_detection = self.detect_poles_from_depth(binary_image, depth_mm)
            
            if pole_detection is not None:
                cx, cy = pole_detection['center_x'], pole_detection['center_y']
                depth_m = pole_detection['depth_m']
                
                X_out, Y_out = self.calculate_3d_position(cx, cy, depth_m)
                filtered_x, filtered_y = self.apply_low_pass_filter(X_out, Y_out)
                
                # 条形码扫描
                barcode_data = ""
                if self.barcode_scan_enabled:
                    try:
                        barcode_results = self.scan_barcode(color_image, cx, cy)
                        if barcode_results:
                            self.publish_barcode_content(barcode_results)
                            
                            barcode_info = []
                            for barcode in barcode_results:
                                if barcode['data'].isdigit() and len(barcode['data']) == 4:
                                    barcode_info.append(barcode['data'])
                            barcode_data = "|".join(barcode_info)
                            
                            if barcode_data:
                                self.get_logger().info(f'杆检测到条形码内容: {barcode_data}')
                    except Exception as e:
                        self.get_logger().error(f'条形码扫描失败: {str(e)}')
                
                # 创建并发布检测消息
                color_msg = Color()
                color_msg.color = "pole"
                color_msg.delta_x = float(filtered_x)
                color_msg.delta_y = float(filtered_y)
                color_msg.detected = True
                
                self.detections_publisher.publish(color_msg)
                
                # 在图像上绘制结果
                self.draw_detection_result(color_image, pole_detection, filtered_x, filtered_y, depth_m, barcode_data)
                
            else:
                self.reset_filter()
                self.publish_empty_detection()
            
            self.draw_task_status(color_image)
            cv2.imshow('Final Result', color_image)
            cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
            traceback.print_exc()
            self.publish_empty_detection()
    
    def draw_task_status(self, image):
        """在图像上绘制任务状态"""
        current_task = self.vision_task_manager.get_task()
        is_active = self.vision_task_manager.is_task_active(1)
        
        status_text = f"Task: {current_task} | This Task: 2 | Status: {'ACTIVE' if is_active else 'INACTIVE'}"
        status_color = (0, 255, 0) if is_active else (0, 0, 255)
        
        text_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.rectangle(image, (5, 5), (text_size[0] + 15, 35), (0, 0, 0), -1)
        cv2.putText(image, status_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        barcode_status = f"Published Barcodes: {len(self.published_barcodes)}"
        cv2.putText(image, barcode_status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    def calculate_3d_position(self, cx, cy, depth_m):
        """计算3D位置"""
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx_cam = self.camera_matrix[0, 2]
        cy_cam = self.camera_matrix[1, 2]
        
        X = (cx - cx_cam) * depth_m / fx
        Y = (cy - cy_cam) * depth_m / fy
        Z = depth_m
        
        X_out = Z
        Y_out = -X
        
        return X_out, Y_out
    
    def draw_detection_result(self, image, pole_info, filtered_x, filtered_y, depth_m, barcode_data=""):
        """在图像上绘制检测结果"""
        cx, cy = pole_info['center_x'], pole_info['center_y']
        detection_method = pole_info['detection_method']
        
        # 绘制中心点
        cv2.circle(image, (cx, cy), 8, (0, 255, 0), -1)
        
        if detection_method == 'line_detection':
            # 绘制线段检测结果
            line1 = pole_info['line1']
            line2 = pole_info['line2']
            
            cv2.line(image, (line1['x1'], line1['y1']), (line1['x2'], line1['y2']), (0, 255, 0), 3)
            cv2.line(image, (line2['x1'], line2['y1']), (line2['x2'], line2['y2']), (0, 255, 0), 3)
            cv2.line(image, (line1['center_x'], line1['center_y']), 
                    (line2['center_x'], line2['center_y']), (255, 0, 0), 2)
            
            text1 = f"Pole (Lines): X={filtered_x:.3f}m, Y={filtered_y:.3f}m, D={depth_m:.3f}m"
            text2 = f"Width:{pole_info['width']:.1f}px Overlap:{pole_info['overlap_length']:.1f}px"
        else:
            # 绘制轮廓检测结果
            if 'contour' in pole_info:
                cv2.drawContours(image, [pole_info['contour']], 0, (0, 255, 0), 2)
            if 'bbox' in pole_info:
                x, y, w, h = pole_info['bbox']
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            text1 = f"Pole (Contour): X={filtered_x:.3f}m, Y={filtered_y:.3f}m, D={depth_m:.3f}m"
            text2 = f"Perimeter:{pole_info.get('perimeter', 0):.1f}px AR:{pole_info.get('aspect_ratio', 0):.2f}"
        
        if barcode_data:
            text1 += f" | Barcode: {barcode_data}"
        
        cv2.putText(image, text1, (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(image, text2, (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        if barcode_data:
            cv2.rectangle(image, (cx - 50, cy - 30), (cx + 200, cy - 10), (0, 255, 0), -1)
            cv2.putText(image, f"Barcode: {barcode_data}", (cx - 45, cy - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    def publish_empty_detection(self):
        """发布空的检测结果"""
        color_msg = Color()
        color_msg.color = "pole"
        color_msg.delta_x = 0.0
        color_msg.delta_y = 0.0
        color_msg.detected = False
        
        self.detections_publisher.publish(color_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        from .vision_task_manager import init_vision_task_manager
        task_subscriber = init_vision_task_manager()
        
        detector = DepthBasedPoleDetector()
        
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(task_subscriber)
        executor.add_node(detector)
        
        executor.spin()
        
    except Exception as e:
        print(f"发生错误: {str(e)}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()