#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msg_tool.msg import Line  # 导入自定义消息
from cv_bridge import CvBridge
import cv2
import numpy as np


class DownCameraNode(Node):
    def __init__(self):
        super().__init__('down_camera_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Create publishers
        self.line_position_pub = self.create_publisher(Line, '/line_detection', 10)  # 使用自定义消息
        
        self.get_logger().info('Down camera node initialized')

        self.width = 640
        self.height = 480
        
        # 添加过滤器变量 - 修改为使用机身坐标系的变量名
        self.last_position_dy = 0.0  # 机身坐标系y方向
        self.last_yaw = 0.0
        self.position_history_dy = []  # 机身坐标系y方向历史
        self.yaw_history = []
        self.history_size = 5
        self.max_jump_threshold = 0.3
        self.last_valid_detection_time = None
        self.detection_timeout = 1.0

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = frame.copy()

            # 获取原始图像尺寸
            orig_height, orig_width = cv_image.shape[:2]
            
            # 计算裁剪区域（居中裁剪）
            target_width, target_height = 480, 480
            start_x = max(0, int((orig_width - target_width) / 2))
            start_y = max(0, int((orig_height - target_height) / 2))
            
            # 裁剪图像
            cv_image = cv_image[start_y:start_y+target_height, start_x:start_x+target_width]
            
            # 确保裁剪后的图像尺寸正确（如果原图小于目标尺寸，进行缩放）
            if cv_image.shape[0] != target_height or cv_image.shape[1] != target_width:
                cv_image = cv2.resize(cv_image, (target_width, target_height))
                
            # 记录裁剪操作
            self.get_logger().debug(f"将图像从 {orig_width}x{orig_height} 裁剪到 {target_width}x{target_height}")

            # Get image dimensions early
            height, width = frame.shape[:2]
            self.height = target_height
            self.width = target_width
            
            # Process the frame for line detection
            processed_frame, binary_image, line_position, line_detected = self.detect_black_line(cv_image)
            
            # 获取当前时间
            current_time = self.get_clock().now()
            
            # Publish line position if detected
            position_msg = Line()
            if line_detected and line_position is not None:
                # 解包相机坐标系的值
                camera_x, camera_yaw = line_position
                
                # 转换到机身坐标系
                # 相机x轴 -> 机身y轴的负方向，相机y轴 -> 机身x轴的负方向
                position_dy = -camera_x  # 相机的x轴对应机身的y轴负方向
                position_yaw = camera_yaw  # yaw角度可能也需要调整，视安装方向而定
                
                # 更新最后有效检测时间
                self.last_valid_detection_time = current_time
                
                # 机身坐标系Y方向平滑处理
                if self.position_history_dy:
                    # 检查跳变
                    avg_position_dy = sum(self.position_history_dy) / len(self.position_history_dy)
                    if abs(position_dy - avg_position_dy) > self.max_jump_threshold:
                        # 如果跳变太大，使用历史平均值
                        position_dy = avg_position_dy
                    
                    # 进行平滑处理
                    self.position_history_dy.append(position_dy)
                    if len(self.position_history_dy) > self.history_size:
                        self.position_history_dy.pop(0)
                    
                    # 计算平滑后的位置
                    smoothed_position_dy = sum(self.position_history_dy) / len(self.position_history_dy)
                    position_dy = smoothed_position_dy
                else:
                    # 第一次检测，直接添加到历史记录
                    self.position_history_dy.append(position_dy)
                
                # Yaw角度平滑处理
                if self.yaw_history:
                    # 添加到历史记录
                    self.yaw_history.append(position_yaw)
                    if len(self.yaw_history) > self.history_size:
                        self.yaw_history.pop(0)
                    
                    # 计算平滑后的yaw
                    smoothed_yaw = sum(self.yaw_history) / len(self.yaw_history)
                    position_yaw = smoothed_yaw
                else:
                    # 第一次检测，直接添加到历史记录
                    self.yaw_history.append(position_yaw)
                
                # 发布消息使用机身坐标系
                position_msg.dy = position_dy
                position_msg.yaw = position_yaw
                position_msg.detected = True
                
                # 更新最后有效位置
                self.last_position_dy = position_dy
                self.last_yaw = position_yaw
                
                # 打印检测到的线位置日志
                # self.get_logger().info(f'Line detected at dy: {position_msg.dy:.2f}, yaw: {position_msg.yaw:.2f}')
            else:
                # 检查是否在超时时间内有过有效检测
                if (self.last_valid_detection_time is not None and 
                    (current_time - self.last_valid_detection_time).nanoseconds / 1e9 < self.detection_timeout):
                    # 在超时时间内，使用最后的有效位置
                    position_msg.dy = self.last_position_dy
                    position_msg.yaw = self.last_yaw
                    position_msg.detected = True
                else:
                    # 超过超时时间，重置位置和历史记录
                    position_msg.dy = 0.0
                    position_msg.yaw = 0.0
                    position_msg.detected = False
                    self.position_history_dy = []
                    self.yaw_history = []
            
            self.line_position_pub.publish(position_msg)
            
            # 显示处理后的图像和二值化图像
            cv2.imshow('Processed Frame', processed_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_black_line(self, frame):
        """
        First use Hough transform to detect lines, if that fails use color-based contour detection
        """
        # Save original frame for display
        original_frame = frame.copy()
        height, width = frame.shape[:2]
    
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        # Binary thresholding to isolate black lines
        _, binary = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
        
        # 保存二值化图像用于显示
        binary_image = binary.copy()
    
        # Morphological operations to remove noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=20)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=3)
    
        # Edge detection
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    
        # Use Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=80, maxLineGap=50)
    
        # Initialize black line center point and yaw
        line_position = None
        line_detected = False
        yaw = 0.0
        
        # 计算画面中心点
        center_of_frame_x = width / 2
        center_of_frame_y = height / 2
    
        # Method 1: Hough Line Detection
        if lines is not None and len(lines) > 0:
            line_detected = True
    
            # Find the longest line
            max_length = 0
            longest_line = None
    
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
                if length > max_length:
                    max_length = length
                    longest_line = (x1, y1, x2, y2)
    
            if longest_line:
                x1, y1, x2, y2 = longest_line
                
                # Calculate line center point
                absolute_center_x = (x1 + x2) / 2
                absolute_center_y = (y1 + y2) / 2
                
                # 计算相对于画面中心的偏移量并归一化
                center_x = (absolute_center_x - center_of_frame_x) * 1.50 / 480
                # 不再需要计算 center_y
                
                # 计算偏航角
                yaw = np.arctan2(x2 - x1, y2 - y1)

                # 如果角度大于pi/2或小于-pi/2，将其调整到正确的范围
                if yaw > np.pi/2:
                    yaw = yaw - np.pi
                elif yaw < -np.pi/2:
                    yaw = yaw + np.pi
                
                # 将坐标打包为元组
                line_position = (center_x, yaw)
                
                # 在原始图像上绘制检测到的线和中心点
                cv2.line(original_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(original_frame, (int(absolute_center_x), int(absolute_center_y)), 5, (0, 0, 255), -1)
                
                # 绘制画面中心线
                cv2.line(original_frame, (int(center_of_frame_x), 0), (int(center_of_frame_x), height), (255, 0, 0), 1)
                cv2.line(original_frame, (0, int(center_of_frame_y)), (width, int(center_of_frame_y)), (255, 0, 0), 1)
    
                # Add line information
                angle = yaw * 180 / np.pi
                cv2.putText(original_frame, f"Hough detection: {angle:.1f}°, length: {max_length:.1f}px",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # 显示偏移量信息 - 使用机身坐标系标签
                body_dy = -center_x  # 显示转换后的机身坐标
                cv2.putText(original_frame, f"Body Frame: dy={body_dy:.2f}, Yaw={yaw:.2f}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Add processed image in top right corner for reference
                h, w = original_frame.shape[:2]
                small_h, small_w = h // 4, w // 4
                small_mask = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), (small_w, small_h))
                original_frame[0:small_h, w - small_w:w] = small_mask
    
        return original_frame, binary_image, line_position, line_detected


def main(args=None):
    rclpy.init(args=args)
    
    down_camera_node = DownCameraNode()
    
    rclpy.spin(down_camera_node)
    
    down_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()