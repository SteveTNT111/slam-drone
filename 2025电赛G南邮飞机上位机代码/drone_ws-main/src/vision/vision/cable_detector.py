#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
from geometry_msgs.msg import Point
import tf2_ros
import math
import time

# 自定义消息
# float64 delta_x
# float64 delta_y
# bool detected
from msg_tool.msg import Detect

class CodeDetector(Node):
    def __init__(self):
        super().__init__('code_detector_node')
        
        # 声明参数
        self.declare_parameter('yellow_lower', [20, 70, 70])
        self.declare_parameter('yellow_upper', [35, 255, 255])
        
        # 深度范围参数 (单位:米) - 只用于限制视野
        self.declare_parameter('min_depth', 0)
        self.declare_parameter('max_depth', 1.0)
        
        # 获取参数
        self.yellow_lower = np.array(self.get_parameter('yellow_lower').value)
        self.yellow_upper = np.array(self.get_parameter('yellow_upper').value)
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        
        # 图像中心点坐标（动态获取）
        self.image_center_x = None
        self.image_center_y = None
        
        # 创建发布者
        self.code_pub = self.create_publisher(Image, '/code_image', 10)
        self.yellow_pub = self.create_publisher(Detect, '/cable_detector/yellow', 10)
        
        # 创建CV桥
        self.bridge = CvBridge()
        
        # 创建订阅者 - 订阅RGB和深度图像
        self.rgb_sub = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', 
            self.rgb_callback, 
            10
        )
        
        # 深度图像订阅
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        # 存储最新的RGB和深度图像
        self.latest_rgb = None
        self.latest_depth = None
        self.depth_mask = None  # 用于存储深度遮罩
        
        # 创建tf缓冲区用于坐标变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 条码相关变量
        self.barcode_center_threshold = 1000  # 条码中心与图像中心的距离阈值，小于此值视为居中
        self.snapshot_count = 0  # 拍摄的照片计数
        self.max_snapshots = 3   # 最大拍摄数量
        self.last_snapshot_time = 0  # 上次拍照时间
        self.snapshot_interval = 0.5  # 拍照间隔(秒)
        self.is_taking_snapshots = False  # 是否正在拍照序列中
        self.current_code_type = None  # 当前正在拍摄的码类型
        
        # 新增：黄色物体相关拍照变量
        self.yellow_center_threshold = 800  # 黄色物体中心与图像中心的距离阈值，小于此值视为居中
        self.yellow_snapshot_count = 0  # 拍摄的黄色物体照片计数
        self.max_yellow_snapshots = 3   # 黄色物体最大拍摄数量
        self.last_yellow_snapshot_time = 0  # 上次拍摄黄色物体时间
        self.yellow_snapshot_interval = 0.5  # 黄色物体拍照间隔(秒)
        self.is_taking_yellow_snapshots = False  # 是否正在拍摄黄色物体照片序列中
        
        # 创建显示窗口
        cv2.namedWindow('Code Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Yellow Regions', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth Visualization', cv2.WINDOW_NORMAL)
        cv2.namedWindow('黄色物体截图', cv2.WINDOW_NORMAL)  # 新增：黄色物体截图窗口
        
        self.get_logger().info(f'Code detector node initialized with depth range: {self.min_depth}-{self.max_depth}m')
    
    def rgb_callback(self, rgb_msg):
        """处理RGB图像回调"""
        try:
            # 转换ROS消息为OpenCV格式
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            
            # 检查图像是否成功转换
            if self.latest_rgb is None:
                self.get_logger().warning("接收到的RGB图像为空")
                return
            
            # 动态更新图像中心点坐标
            height, width = self.latest_rgb.shape[:2]
            self.image_center_x = width // 2
            self.image_center_y = height // 2

            self.get_logger().debug(f"图像中心坐标: ({self.image_center_x}, {self.image_center_y})")
            
            # 如果深度图像还未接收，等待
            if self.latest_depth is None:
                return
            
            # 创建深度掩码
            self.create_depth_mask()
            
            # 直接使用原始RGB图像进行扫码，但只在深度范围内的区域进行处理
            # 创建一个可视化图像
            visual_img = self.latest_rgb.copy()
            
            # 处理图像以检测黄色凸起物（在深度限制范围内）
            yellow_regions, yellow_visual = self.detect_yellow_protrusions(self.latest_rgb, self.depth_mask)
            
            # 初始化消息
            protrusion_msg = Detect()
            protrusion_msg.delta_x = 0.0
            protrusion_msg.delta_y = 0.0
            protrusion_msg.detected = False
            
            # 如果找到黄色区域，更新消息
            if yellow_regions and self.image_center_x is not None and self.image_center_y is not None:
                # 以第一个检测到的黄色区域为准
                x, y, w, h = yellow_regions[0]
                
                # 计算区域中心
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 计算相对于图像中心的偏移量
                delta_x = center_x - self.image_center_x
                delta_y = center_y - self.image_center_y
                
                # 更新消息
                protrusion_msg.delta_x = float(delta_x)
                protrusion_msg.delta_y = float(delta_y)
                protrusion_msg.detected = True
                
                self.get_logger().debug(f"黄色区域中心: ({center_x}, {center_y}), 偏移量: ({delta_x}, {delta_y})")
                
                # 新增：检查黄色物体是否位于画面中心
                distance_to_center = math.sqrt(delta_x**2 + delta_y**2)
                if distance_to_center < self.yellow_center_threshold:
                    self.take_yellow_snapshot(self.latest_rgb, (x, y, w, h))
            
            # 直接在原始RGB图像上扫描所有条形码和二维码
            self.scan_all_codes(self.latest_rgb)
            
            # 发布消息
            self.yellow_pub.publish(protrusion_msg)
            
            # 显示黄色区域检测结果
            cv2.imshow('Yellow Regions', yellow_visual)
            
            # 显示原始RGB图像处理结果
            cv2.imshow('Code Detection', visual_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in RGB callback: {str(e)}')
            self.get_logger().debug('Debug info - latest_rgb type: %s', type(self.latest_rgb))
            import traceback
            self.get_logger().debug('Full traceback: %s', traceback.format_exc())
    
    def depth_callback(self, depth_msg):
        """处理深度图像回调"""
        try:
            # 转换ROS消息为OpenCV格式
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {str(e)}')
    
    def create_depth_mask(self):
        """创建基于深度范围的掩码，仅用于限制视野范围"""
        if self.latest_depth is None:
            return
        
        # 创建深度图像的副本用于处理
        depth_processed = self.latest_depth.copy().astype(np.float32)
        
        # 将无效深度值（0或NaN）标记为特殊值
        special_value = 2000.0  # 远大于正常深度范围的值
        invalid_mask = (depth_processed == 0) | np.isnan(depth_processed)
        depth_processed[invalid_mask] = special_value
        
        # 根据深度范围确定有效区域
        min_depth_mm = self.min_depth * 1000.0
        max_depth_mm = self.max_depth * 1000.0
        
        # 创建掩码：只包括min_depth到max_depth范围内的像素
        in_range = (depth_processed >= min_depth_mm) & (depth_processed <= max_depth_mm)
        self.depth_mask = in_range.astype(np.uint8)
        
        # 创建可视化的深度图像（用于调试）
        norm_depth = np.zeros_like(depth_processed, dtype=np.uint8)
        
        # 对有效范围内的深度值进行归一化（最近处为0，最远处接近255）
        if np.any(in_range):
            # 获取有效范围内的最小值和最大值
            valid_depths = depth_processed[in_range]
            min_valid = np.min(valid_depths) if len(valid_depths) > 0 else min_depth_mm
            max_valid = np.max(valid_depths) if len(valid_depths) > 0 else max_depth_mm
            
            if max_valid > min_valid:  # 避免除以零
                # 归一化有效范围内的深度值
                scaled = ((depth_processed[in_range] - min_valid) * 255 / (max_valid - min_valid))
                norm_depth[in_range] = scaled.astype(np.uint8)
        
        # 将大于max_depth的区域设置为255（白色）
        too_far_mask = depth_processed > max_depth_mm
        norm_depth[too_far_mask] = 255
        
        # 创建彩色深度图用于可视化
        depth_visual = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
        cv2.imshow('Depth Visualization', depth_visual)
        
        self.get_logger().debug(f"深度掩码创建完成，有效像素数量: {np.sum(self.depth_mask)}")
    
    def detect_yellow_protrusions(self, color_image, depth_mask=None):
        """检测黄色凸起物并返回可视化图像"""
        # 创建副本用于可视化
        visual_img = color_image.copy()
        
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 创建黄色掩码
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # 如果有深度掩码，应用深度限制
        if depth_mask is not None:
            yellow_mask = cv2.bitwise_and(yellow_mask, yellow_mask, mask=depth_mask)
        
        # 对掩码进行形态学操作
        kernel = np.ones((3,3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 寻找黄色区域轮廓
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 过滤掉过小的轮廓
        min_area = 5000  # 最小面积阈值
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        
        # 为每个黄色区域获取边界框并在可视化图像上绘制
        yellow_regions = []
        for cnt in filtered_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            yellow_regions.append((x, y, w, h))
            # 在可视化图像上绘制黄色区域
            # cv2.rectangle(visual_img, (x, y), (x+w, y+h), (0, 255, 255), 2)
            # cv2.putText(visual_img, "Yellow Region", (x, y-10), 
            #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # 添加中心点标记
            # center_x = x + w // 2
            # center_y = y + h // 2
            # cv2.circle(visual_img, (center_x, center_y), 5, (0, 0, 255), -1)
            
        # 在图像上绘制中心十字线
        # if self.image_center_x is not None and self.image_center_y is not None:
        #     cv2.line(visual_img, (self.image_center_x - 20, self.image_center_y), 
        #              (self.image_center_x + 20, self.image_center_y), (255, 255, 255), 2)
        #     cv2.line(visual_img, (self.image_center_x, self.image_center_y - 20), 
        #              (self.image_center_x, self.image_center_y + 20), (255, 255, 255), 2)
            
        return yellow_regions, visual_img
    
    def take_yellow_snapshot(self, color_image, yellow_region):
        """当黄色物体位于画面中心时拍摄照片"""
        try:
            current_time = time.time()
            x, y, w, h = yellow_region
            
            # 如果不在拍照序列中，初始化拍照序列
            if not self.is_taking_yellow_snapshots:
                self.is_taking_yellow_snapshots = True
                self.yellow_snapshot_count = 0
                self.get_logger().info("开始黄色物体拍照序列")
            
            # 检查是否可以拍照（时间间隔和最大数量）
            if (current_time - self.last_yellow_snapshot_time >= self.yellow_snapshot_interval and 
                    self.yellow_snapshot_count < self.max_yellow_snapshots):
                
                # 增加计数并更新时间
                self.yellow_snapshot_count += 1
                self.last_yellow_snapshot_time = current_time
                
                # 使用原始RGB图像的副本
                full_image = self.latest_rgb.copy()
                
                # 在图像上标记黄色区域位置
                cv2.rectangle(full_image, (x, y), (x+w, y+h), (0, 255, 255), 3)
                
                # 在图像上添加信息（放在左上角）
                # info_text = [
                #     "类型: 黄色物体",
                #     "位置: 画面中心",
                #     f"拍照 {self.yellow_snapshot_count}/{self.max_yellow_snapshots}"
                # ]
                
                # 文字背景矩形的高度
                # bg_height = len(info_text) * 25 + 10
                
                # # 绘制半透明背景
                # overlay = full_image.copy()
                # cv2.rectangle(overlay, (10, 10), (400, 10 + bg_height), (0, 0, 0), -1)
                # cv2.addWeighted(overlay, 0.6, full_image, 0.4, 0, full_image)
                
                # # 添加文本信息
                # for i, text in enumerate(info_text):
                #     cv2.putText(full_image, text, (15, 35 + i * 25), 
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                #                 (0, 255, 255) if i == 2 else (0, 255, 0), 2)
                
                # 添加中心十字线
                # cv2.line(full_image, (self.image_center_x - 20, self.image_center_y), 
                #          (self.image_center_x + 20, self.image_center_y), (255, 255, 255), 2)
                # cv2.line(full_image, (self.image_center_x, self.image_center_y - 20), 
                #          (self.image_center_x, self.image_center_y + 20), (255, 255, 255), 2)
                
                # 显示拍照图像
                window_title = "黄色物体截图"
                cv2.imshow(window_title, full_image)
                cv2.waitKey(1)
                
                # 发布完整图像
                self.code_pub.publish(self.bridge.cv2_to_imgmsg(full_image, "bgr8"))
                self.get_logger().info(f"黄色物体截图 {self.yellow_snapshot_count}/{self.max_yellow_snapshots}")
                
                # 检查是否完成拍照序列
                if self.yellow_snapshot_count >= self.max_yellow_snapshots:
                    self.is_taking_yellow_snapshots = False
                    self.get_logger().info("完成黄色物体拍照序列")
                    
        except Exception as e:
            self.get_logger().error(f"拍摄黄色物体照片时出错: {str(e)}")
    
    def scan_all_codes(self, color_image):
        """扫描整个RGB图像中的所有条形码和二维码"""
        # 扫描所有条码
        all_codes = pyzbar.decode(color_image)
        
        for code in all_codes:
            try:
                # 获取条码位置和数据
                (x, y, w, h) = code.rect
                
                # 检查宽度和高度是否为零
                if w == 0 or h == 0:
                    self.get_logger().warning("检测到条码宽度或高度为零，跳过此条码")
                    continue
                
                # 检查条码位置是否在图像范围内
                if (x < 0 or y < 0 or 
                    x + w > color_image.shape[1] or 
                    y + h > color_image.shape[0]):
                    self.get_logger().warning("检测到条码超出图像范围，跳过此条码")
                    continue
                    
                code_data = code.data.decode('utf-8')
                code_type = code.type
                
                # 计算条码中心点
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 创建条码ROI
                code_roi = color_image[y:y+h, x:x+w].copy()
                
                # 判断条码类型并进行相应处理
                if code_type == 'QRCODE':
                    # 处理二维码
                    cv2.rectangle(code_roi, (0, 0), (w, h), (255, 0, 0), 2)
                    cv2.putText(code_roi, f"二维码: {code_data}", (0, h + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    self.get_logger().info(f"发现二维码: {code_data}")
                else:
                    # 处理条形码
                    cv2.rectangle(code_roi, (0, 0), (w, h), (0, 255, 0), 2)
                    cv2.putText(code_roi, f"条形码: {code_data}", (0, h + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    self.get_logger().info(f"发现条形码: {code_data}, 类型: {code_type}")
                
                # 检查条码是否接近图像中心（确保图像中心坐标已初始化）
                if self.image_center_x is not None and self.image_center_y is not None:
                    distance_to_center = math.sqrt(
                        (center_x - self.image_center_x)**2 + 
                        (center_y - self.image_center_y)**2
                    )
                else:
                    continue  # 如果图像中心还未初始化，跳过此条码
                
                # 如果条码接近图像中心，拍摄照片
                if distance_to_center < self.barcode_center_threshold:
                    code_type_name = "qrcode" if code_type == 'QRCODE' else "barcode"
                    self.take_code_snapshot(color_image, code, code_type_name)
                    
            except Exception as e:
                self.get_logger().error(f"处理条码时出错: {str(e)}")
                continue
    
    def take_code_snapshot(self, color_image, code, code_type):
        """当条码/二维码在图像中心时拍摄照片，使用原始RGB图像"""
        try:
            current_time = time.time()
            
            # 如果不在拍照序列中，或者正在拍摄不同类型的码，初始化拍照序列
            if not self.is_taking_snapshots or self.current_code_type != code_type:
                self.is_taking_snapshots = True
                self.snapshot_count = 0
                self.current_code_type = code_type
                self.get_logger().info(f"开始{code_type}拍照序列")
            
            # 检查是否可以拍照（时间间隔和最大数量）
            if (current_time - self.last_snapshot_time >= self.snapshot_interval and 
                    self.snapshot_count < self.max_snapshots):
                
                # 增加计数并更新时间
                self.snapshot_count += 1
                self.last_snapshot_time = current_time
                
                # 获取条码区域用于标记，但不裁剪图像
                (x, y, w, h) = code.rect
                
                # 检查宽度和高度是否为零
                if w == 0 or h == 0:
                    self.get_logger().warning("条码宽度或高度为零，无法拍照")
                    return
                
                # 检查条码位置是否在图像范围内
                if (x < 0 or y < 0 or 
                    x + w > self.latest_rgb.shape[1] or 
                    y + h > self.latest_rgb.shape[0]):
                    self.get_logger().warning("条码超出图像范围，无法拍照")
                    return
                
                # 使用原始RGB图像的副本，而不是传入的处理过的图像
                full_image = self.latest_rgb.copy()
                
                # 在图像上标记条码位置
                cv2.rectangle(full_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # 获取码数据
                code_data = code.data.decode('utf-8')
                code_type_name = "二维码" if code_type == "qrcode" else "条形码"
                
                # 在图像上添加信息（放在左上角）
                info_text = [
                    f"类型: {code_type_name}",
                    f"数据: {code_data}",
                    f"拍照 {self.snapshot_count}/{self.max_snapshots}"
                ]
                
                # 文字背景矩形的高度
                bg_height = len(info_text) * 25 + 10
                
                # 绘制半透明背景
                overlay = full_image.copy()
                cv2.rectangle(overlay, (10, 10), (400, 10 + bg_height), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.6, full_image, 0.4, 0, full_image)
                
                # 添加文本信息
                for i, text in enumerate(info_text):
                    cv2.putText(full_image, text, (15, 35 + i * 25), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                                (0, 255, 255) if i == 2 else (0, 255, 0), 2)
                
                # 显示拍照图像
                window_title = "码扫描截图"
                cv2.imshow(window_title, full_image)
                cv2.waitKey(1)
                
                # 使用统一的话题发布完整图像
                self.code_pub.publish(self.bridge.cv2_to_imgmsg(full_image, "bgr8"))
                self.get_logger().info(f"{code_type_name}截图 {self.snapshot_count}/{self.max_snapshots}: {code_data}")
                
                # 检查是否完成拍照序列
                if self.snapshot_count >= self.max_snapshots:
                    self.is_taking_snapshots = False
                    self.current_code_type = None
                    self.get_logger().info(f"完成{code_type_name}拍照序列")
                    
        except Exception as e:
            self.get_logger().error(f"拍摄条码照片时出错: {str(e)}")

    def destroy_node(self):
        """重载销毁节点方法，关闭所有OpenCV窗口"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detector = CodeDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()