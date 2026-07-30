#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import traceback

from .color_detector_core import ColorDetector, CalculateCoordinates, LowPassFilter, ColorSpace
from msg_tool.msg import Pole

# 定义参数文件，此文件将只保存HSV值
PARAMS_FILE = 'pole_hsv_params.json'

class VerticalPoleDetector(Node):
    def __init__(self):
        super().__init__('vertical_pole_detector')

        # ----------- 1. 参数管理 -----------
        self.hsv_params = {}
        self.load_hsv_params()

        # 固定参数
        self.area_threshold = 1000
        self.close_iter = 5
        self.open_iter = 3
        self.min_depth = 0.0
        self.max_depth = 1.0
        self.filter_alpha = 0.7

        # UI和模式管理
        self.trackbar_window_name = 'HSV Tuning Control' # 改回单个窗口名称
        self.current_mode = 0
        self.trackbars_visible = False

        # ----------- 2. 初始化检测器和工具 -----------
        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None
        self.color_detector = ColorDetector(target_width=640, target_height=480, enable_preprocessing=True)
        self.camera_matrix = self.color_detector.camera_matrix
        self.coord_calculator = CalculateCoordinates(self.camera_matrix)
        self.low_pass_filter = LowPassFilter(alpha=self.filter_alpha)

        # ----------- 3. ROS 订阅与发布 -----------
        self.depth_subscriber = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.color_subscriber = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Int32, '/task', self.task_callback, 10)
        self.pole_publisher = self.create_publisher(Pole, '/detected_pole', 10)
        self.timer = self.create_timer(0.1, self.process_images)

        self.get_logger().info('竖直杆检测器已初始化，待机模式(2)。等待/task消息...')

    def task_callback(self, msg):
        """根据/task消息更新节点模式，并管理UI"""
        new_mode = msg.data
        if new_mode == self.current_mode:
            return

        self.get_logger().info(f'模式切换: {self.current_mode} -> {new_mode}')
        self.current_mode = new_mode

        # 管理调参窗口的显示和隐藏
        if self.current_mode == 0 and not self.trackbars_visible:
            self.setup_trackbars()
            self.trackbars_visible = True
            self.get_logger().info('进入调参模式(0)，已打开HSV Tuning Control窗口。')
        elif self.current_mode != 0 and self.trackbars_visible:
            cv2.destroyWindow(self.trackbar_window_name) # 只需销毁一个窗口
            self.trackbars_visible = False
            self.get_logger().info('离开调参模式，已关闭调参窗口。')
        
        if self.current_mode == 1:
            self.get_logger().info('进入运行模式(1)，重新加载HSV文件参数...')
            self.load_hsv_params()

    def load_hsv_params(self):
        """仅从JSON文件加载HSV参数"""
        if os.path.exists(PARAMS_FILE):
            try:
                with open(PARAMS_FILE, 'r') as f:
                    self.hsv_params = json.load(f)
                self.get_logger().info(f'成功从 {PARAMS_FILE} 加载HSV参数。')
                return
            except Exception as e:
                self.get_logger().error(f'加载HSV参数文件失败: {e}，将使用默认值。')
        
        self.hsv_params = {
            'h_min1': 0, 's_min1': 120, 'v_min1': 100,
            'h_max1': 10, 's_max1': 255, 'v_max1': 255,
            'h_min2': 170, 's_min2': 120, 'v_min2': 100,
            'h_max2': 180, 's_max2': 255, 'v_max2': 255
        }
        self.get_logger().info('未找到HSV参数文件，已使用默认值。')

    def save_hsv_params(self):
        """仅将当前滑块的HSV参数保存到JSON文件"""
        self.update_hsv_from_trackbars()
        try:
            with open(PARAMS_FILE, 'w') as f:
                json.dump(self.hsv_params, f, indent=4)
            self.get_logger().info(f'HSV参数已成功保存到 {PARAMS_FILE}')
        except Exception as e:
            self.get_logger().error(f'保存HSV参数失败: {e}')

    def on_trackbar_change(self, val):
        pass

    def setup_trackbars(self):
        """创建单个、更宽的滑块窗口"""
        cv2.namedWindow(self.trackbar_window_name)
        
        # 使用更长的名称来加宽窗口
        cv2.createTrackbar('h_min1', self.trackbar_window_name, self.hsv_params['h_min1'], 180, self.on_trackbar_change)
        cv2.createTrackbar('s_min1', self.trackbar_window_name, self.hsv_params['s_min1'], 255, self.on_trackbar_change)
        cv2.createTrackbar('v_min1', self.trackbar_window_name, self.hsv_params['v_min1'], 255, self.on_trackbar_change)
        cv2.createTrackbar('h_max1', self.trackbar_window_name, self.hsv_params['h_max1'], 180, self.on_trackbar_change)
        cv2.createTrackbar('s_max1', self.trackbar_window_name, self.hsv_params['s_max1'], 255, self.on_trackbar_change)
        cv2.createTrackbar('v_max1', self.trackbar_window_name, self.hsv_params['v_max1'], 255, self.on_trackbar_change)
        
        # 添加分隔（视觉上），但OpenCV不支持，只能按顺序排列
        cv2.createTrackbar('h_min2', self.trackbar_window_name, self.hsv_params['h_min2'], 180, self.on_trackbar_change)
        cv2.createTrackbar('s_min2', self.trackbar_window_name, self.hsv_params['s_min2'], 255, self.on_trackbar_change)
        cv2.createTrackbar('v_min2', self.trackbar_window_name, self.hsv_params['v_min2'], 255, self.on_trackbar_change)
        cv2.createTrackbar('h_max2', self.trackbar_window_name, self.hsv_params['h_max2'], 180, self.on_trackbar_change)
        cv2.createTrackbar('s_max2', self.trackbar_window_name, self.hsv_params['s_max2'], 255, self.on_trackbar_change)
        cv2.createTrackbar('v_max2', self.trackbar_window_name, self.hsv_params['v_max2'], 255, self.on_trackbar_change)
        
        cv2.createTrackbar('---SAVE_PARAMS---', self.trackbar_window_name, 0, 1, self.on_trackbar_change)

    def update_hsv_from_trackbars(self):
        """从单个窗口中读取所有HSV值"""
        self.hsv_params['h_min1'] = cv2.getTrackbarPos('Range1_Hue_Min', self.trackbar_window_name)
        self.hsv_params['s_min1'] = cv2.getTrackbarPos('Range1_Sat_Min', self.trackbar_window_name)
        self.hsv_params['v_min1'] = cv2.getTrackbarPos('Range1_Val_Min', self.trackbar_window_name)
        self.hsv_params['h_max1'] = cv2.getTrackbarPos('Range1_Hue_Max', self.trackbar_window_name)
        self.hsv_params['s_max1'] = cv2.getTrackbarPos('Range1_Sat_Max', self.trackbar_window_name)
        self.hsv_params['v_max1'] = cv2.getTrackbarPos('Range1_Val_Max', self.trackbar_window_name)

        self.hsv_params['h_min2'] = cv2.getTrackbarPos('Range2_Hue_Min', self.trackbar_window_name)
        self.hsv_params['s_min2'] = cv2.getTrackbarPos('Range2_Sat_Min', self.trackbar_window_name)
        self.hsv_params['v_min2'] = cv2.getTrackbarPos('Range2_Val_Min', self.trackbar_window_name)
        self.hsv_params['h_max2'] = cv2.getTrackbarPos('Range2_Hue_Max', self.trackbar_window_name)
        self.hsv_params['s_max2'] = cv2.getTrackbarPos('Range2_Sat_Max', self.trackbar_window_name)
        self.hsv_params['v_max2'] = cv2.getTrackbarPos('Range2_Val_Max', self.trackbar_window_name)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {str(e)}')

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'处理彩色图像时出错: {str(e)}')

    def process_images(self):
        if self.current_mode == 2 or self.depth_image is None or self.color_image is None:
            cv2.waitKey(1)
            return

        try:
            if self.current_mode == 0:
                if not self.trackbars_visible: self.setup_trackbars(); self.trackbars_visible = True
                self.update_hsv_from_trackbars()
                if cv2.getTrackbarPos('---SAVE_PARAMS---', self.trackbar_window_name) == 1:
                    self.save_hsv_params()
                    cv2.setTrackbarPos('---SAVE_PARAMS---', self.trackbar_window_name, 0)
            
            color_img = self.color_image.copy()
            depth_img = self.depth_image.copy()

            red_thresh = [
                [self.hsv_params['h_min1'], self.hsv_params['s_min1'], self.hsv_params['v_min1']],
                [self.hsv_params['h_max1'], self.hsv_params['s_max1'], self.hsv_params['v_max1']],
                [self.hsv_params['h_min2'], self.hsv_params['s_min2'], self.hsv_params['v_min2']],
                [self.hsv_params['h_max2'], self.hsv_params['s_max2'], self.hsv_params['v_max2']]
            ]

            result = self.color_detector.detect(
                color_img, colors='red', red_threshold=red_thresh,
                area_threshold=self.area_threshold, close_iterations=self.close_iter, open_iterations=self.open_iter
            )

            # 后续逻辑与之前完全相同
            if not result.detected:
                self.publish_pole_position(0.0, 0.0, False)
                self.low_pass_filter.reset()
                cv2.imshow('Detection Result', color_img)
                cv2.waitKey(1)
                return

            cx, cy = result.center_x, result.center_y
            if depth_img.dtype == np.uint16:
                depth_m = float(depth_img[cy, cx]) / 1000.0
            else:
                depth_m = float(depth_img[cy, cx])

            if not (self.min_depth < depth_m < self.max_depth):
                self.publish_pole_position(0.0, 0.0, False)
                debug_img_on_fail = self.color_detector.draw_results(color_img, [result])
                cv2.imshow('Detection Result', debug_img_on_fail)
                cv2.waitKey(1)
                return
            
            X_out, Y_out, _ = self.coord_calculator.front_calculate_coordinates(cx, cy, depth_m)
            f_x, f_y, _ = self.low_pass_filter.filter(X_out, Y_out, 0.0, True)
            self.publish_pole_position(f_x, f_y, True)
            
            debug_img = self.color_detector.draw_results(color_img, [result])
            cv2.putText(debug_img, f"X: {f_x:.3f}m, Y: {f_y:.3f}m", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_img, f"Depth: {depth_m:.3f}m", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.imshow('Detection Result', debug_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
            traceback.print_exc()
            self.publish_pole_position(0.0, 0.0, False)
            self.low_pass_filter.reset()

    def publish_pole_position(self, x, y, detected):
        try:
            pole_msg = Pole()
            pole_msg.x = float(x)
            pole_msg.y = float(y)
            pole_msg.detected = bool(detected)
            self.pole_publisher.publish(pole_msg)
        except Exception as e:
            self.get_logger().error(f'发布杆位置时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    detector = None
    try:
        detector = VerticalPoleDetector()
        rclpy.spin(detector)
    except Exception as e:
        print(f"发生错误: {str(e)}")
        traceback.print_exc()
    finally:
        if detector:
            detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()