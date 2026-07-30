import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge
import cv2
from .color_detector_core import ColorDetector, ColorDetectionResult

class PlantDetection(Node):
    def __init__(self):
        super().__init__('plant_detection_node')

        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 初始化图像变量
        self.color_image = None

        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 创建与C++发布者匹配的QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # 添加对/arrived话题的订阅
        self.create_subscription(
            Bool,
            '/arrived',
            self.light_callback,
            qos_profile
        )

        # 创建定时器用于图像处理
        self.timer = self.create_timer(0.1, self.process_images)
        self.publisher = self.create_publisher(Int32, '/esp32', 10)

        self.plant_msg = Int32()
        
        self.get_logger().info('初始化成功')

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'处理彩色图像时出错: {str(e)}')

    def light_callback(self, msg):
        """处理/arrived话题的回调函数"""
        self.publisher.publish(self.plant_msg)
        self.get_logger().info('到达点，发送成功')

    def process_images(self):
        # 检查是否有可用的图像
        if self.color_image is None:
            return
        
        detected = 0

        # 在这里添加植物检测的逻辑
        detection = ColorDetector(target_width=160,
                                target_height=160,
                                area_threshold=2000, 
                                coordinate_scale=1.5,
                                )
        results = detection.detect(self.color_image, 
                                   colors=['green'], 
                                   green_threshold=[[60, 50, 100], [80, 255, 255]],
                                   close_iterations=3,
                                   open_iterations=2,                                    
                                   )
        img = detection.draw_results(self.color_image, results)
        cv2.imshow("Detected Image", img)
        cv2.waitKey(1)

        for result in results:
            if not result:
                continue
            green_area = result.contour_area
            if green_area > 20000:
                detected = 1
                # self.get_logger().info('是绿色')
            else:
                detected = 0
        
        self.plant_msg.data = detected

def main(args=None):
    rclpy.init(args=args)
    node = PlantDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  # 清理OpenCV窗口
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()