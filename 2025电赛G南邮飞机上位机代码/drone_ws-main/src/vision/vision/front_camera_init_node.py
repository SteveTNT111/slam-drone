#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FrontCameraInitNode(Node):
    def __init__(self):
        super().__init__('front_camera_init_node')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 创建图像发布者
        self.image_publisher = self.create_publisher(
            Image,
            '/front_camera/image_raw',
            10
        )
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture('/dev/front_camera')  # 使用默认摄像头，根据需要可以更改索引
        
        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # 设置摄像头帧率为30fps
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 检查摄像头是否成功打开
        if not self.cap.isOpened():
            self.get_logger().error('无法打开摄像头')
            return
        
        # 创建定时器，30Hz频率读取和发布图像
        self.timer = self.create_timer(1/30.0, self.timer_callback)
        
        self.get_logger().info('摄像头发布节点初始化成功，分辨率: 640x480，帧率: 30fps')
        
    def timer_callback(self):
        # 读取一帧图像
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('无法读取摄像头帧')
            return
        
        # 将OpenCV图像转换为ROS图像消息
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # 设置时间戳
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_frame'
        
        # 发布图像
        self.image_publisher.publish(ros_image)
        
    def __del__(self):
        # 释放摄像头资源
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    camera_publisher = FrontCameraInitNode()
    
    rclpy.spin(camera_publisher)
    
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()