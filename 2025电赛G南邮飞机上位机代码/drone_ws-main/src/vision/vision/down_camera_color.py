import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from msg_tool.msg import Color, ColorDetections
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        # 图像订阅和结果发布
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(ColorDetections, 'color_detections', 10)
        self.bridge = CvBridge()
        
        # 初始化形态学核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
        # 颜色配置（名称，阈值范围，显示颜色）
        self.color_config = [
            ('red', 
             [np.array([0, 50, 50]), np.array([10, 255, 255]),
              np.array([170, 50, 50]), np.array([180, 255, 255])],
             (0, 0, 255)),  # BGR颜色：红色
            
            ('yellow', 
             [np.array([20, 50, 50]), np.array([35, 255, 255])],
             (0, 255, 255)),  # BGR颜色：黄色
            
            ('green', 
             [np.array([35, 50, 50]), np.array([85, 255, 255])],
             (0, 255, 0))    # BGR颜色：绿色
        ]

    def preprocess_image(self, bgr_img):
        """图像预处理管道"""
        # 双边滤波降噪（保留边缘）
        filtered = cv2.bilateralFilter(bgr_img, d=5, sigmaColor=75, sigmaSpace=75)
        return filtered

    def create_mask(self, hsv_img, color_ranges):
        """创建颜色掩膜并优化"""
        if len(color_ranges) == 4:  # 红色双区间处理
            lower1, upper1, lower2, upper2 = color_ranges
            mask1 = cv2.inRange(hsv_img, lower1, upper1)
            mask2 = cv2.inRange(hsv_img, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:  # 单区间处理
            lower, upper = color_ranges
            mask = cv2.inRange(hsv_img, lower, upper)
        
        # 形态学优化（先闭后开）
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=3)  # 填充小孔洞
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)   # 去除小噪声
        return mask

    def detect_contour(self, mask):
        """轮廓检测与特征计算"""
        detect = False
        cx = cy = 0
        largest_contour = None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # 取最大面积轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    detect = True
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
        return detect, cx, cy, largest_contour

    def image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # 获取原始图像尺寸
            orig_height, orig_width = cv_image.shape[:2]
            
            # 计算裁剪区域（居中裁剪）
            target_width, target_height = 480, 320
            start_x = max(0, int((orig_width - target_width) / 2))
            start_y = max(0, int((orig_height - target_height) / 2))
            
            # 裁剪图像
            cv_image = cv_image[start_y:start_y+target_height, start_x:start_x+target_width]
            
            # 确保裁剪后的图像尺寸正确（如果原图小于目标尺寸，进行缩放）
            if cv_image.shape[0] != target_height or cv_image.shape[1] != target_width:
                cv_image = cv2.resize(cv_image, (target_width, target_height))
                
            # 记录裁剪操作
            self.get_logger().debug(f"将图像从 {orig_width}x{orig_height} 裁剪到 {target_width}x{target_height}")
            
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {str(e)}')
            return

        # 图像预处理
        processed_img = self.preprocess_image(cv_image)
        hsv_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2HSV)
        height, width = cv_image.shape[:2]

        # 创建检测结果消息
        detections_msg = ColorDetections()
        
        # 处理每个颜色配置
        for config in self.color_config:
            color_name = config[0]
            color_ranges = config[1]
            display_color = config[2]

            # 生成优化后的颜色掩膜
            mask = self.create_mask(hsv_img, color_ranges)
            
            # 检测轮廓并计算位置
            detected, cx, cy, largest_contour = self.detect_contour(mask)
            
            # 计算实际偏移量
            delta_x = 0.0
            delta_y = 0.0
            if detected:
                dx = (cx - width/2) * 1.50 / 480
                dy = (cy - height/2) * 1.50 / 480

                delta_x = -dy
                delta_y = -dx
                
                # 记录检测日志
                # self.get_logger().info(
                #     f"检测到 {color_name}: "
                #     f"像素坐标({cx}, {cy}), "
                #     f"实际偏移({delta_x:.2f}m, {delta_y:.2f}m)",
                #     throttle_duration_sec=1  # 限流每秒1条
                # )
                
                # 在图像上绘制检测结果
                cv2.circle(cv_image, (cx, cy), 10, display_color, -1)
                
                # 根据不同颜色显示不同的形状标签
                if color_name == 'yellow':
                    shape_text = "rectangle"
                else:
                    shape_text = "circle"
                
                # 显示颜色名称、形状标签和位置信息
                cv2.putText(cv_image, 
                           f"{color_name} {shape_text}: ({delta_x:.2f}, {delta_y:.2f})",
                           (10, 10),
                           cv2.FONT_HERSHEY_SIMPLEX,
                           0.6, display_color, 2)
                           
                cv2.drawContours(cv_image, [largest_contour], -1, display_color, 2)

            # 填充检测结果
            detection = Color()
            detection.color = color_name
            detection.delta_x = delta_x if detected else 0.0
            detection.delta_y = delta_y if detected else 0.0
            detection.detected = detected
            detections_msg.detections.append(detection)

        # 显示处理结果
        cv2.imshow('Color Detection Preview', cv_image)
        cv2.waitKey(1)

        # 发布检测结果
        self.publisher.publish(detections_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = ColorDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('节点已关闭')
    finally:
        cv2.destroyAllWindows()
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()