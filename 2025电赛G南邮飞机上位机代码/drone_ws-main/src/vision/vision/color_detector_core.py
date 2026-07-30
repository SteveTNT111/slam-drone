import cv2
import numpy as np
import math
from typing import List, Tuple, Dict, Optional, Union
from dataclasses import dataclass
from enum import Enum

class ColorSpace(Enum):
    """色彩空间枚举"""
    HSV = "hsv"
    HLS = "hls"
    BINARY = "binary"

@dataclass
class ColorDetectionResult:
    """颜色检测结果数据类"""
    color: str
    detected: bool
    center_x: int = 0
    center_y: int = 0
    delta_x: float = 0.0
    delta_y: float = 0.0
    contour_area: float = 0.0
    contour: Optional[np.ndarray] = None
    color_space: str = "hsv"  # 添加色彩空间信息

class ColorDetector:
    """颜色检测器类"""
    
    def __init__(self, 
                 target_width: int = 640,
                 target_height: int = 480,
                 area_threshold: int = 10000,
                 coordinate_scale: float = 1.50,
                 close_iterations: int = 3,
                 open_iterations: int = 2,
                 kernel_size: Tuple[int, int] = (3, 3),
                 kernel_shape: int = cv2.MORPH_ELLIPSE,
                 color_thresholds: Optional[Dict] = None,
                 enable_preprocessing: bool = True,
                 binary_threshold: int = 50,
                 default_color_space: ColorSpace = ColorSpace.HSV):
        """
        初始化颜色检测器
        
        Args:
            target_width: 目标图像宽度
            target_height: 目标图像高度
            area_threshold: 轮廓面积阈值
            coordinate_scale: 坐标转换比例
            close_iterations: 闭运算迭代次数
            open_iterations: 开运算迭代次数
            kernel_size: 形态学核大小
            kernel_shape: 形态学核形状
            color_thresholds: 颜色阈值配置
            enable_preprocessing: 是否启用图像预处理
            binary_threshold: 二值化阈值（用于黑色检测）
            default_color_space: 默认色彩空间
        """
        self.target_width = target_width
        self.target_height = target_height
        self.area_threshold = area_threshold
        self.coordinate_scale = coordinate_scale
        self.close_iterations = close_iterations
        self.open_iterations = open_iterations
        self.enable_preprocessing = enable_preprocessing
        self.binary_threshold = binary_threshold
        self.default_color_space = default_color_space
        
        # 创建形态学核
        self.kernel = cv2.getStructuringElement(kernel_shape, kernel_size)
        
        # 设置默认HSV颜色阈值配置
        self.default_hsv_thresholds = {
            'red': {
                'ranges': [
                    np.array([0, 70, 50]), np.array([10, 255, 255]),
                    np.array([170, 70, 50]), np.array([180, 255, 255])
                ],
                'display_color': (0, 0, 255),
                'color_space': ColorSpace.HSV
            },
            'yellow': {
                'ranges': [np.array([20, 70, 70]), np.array([35, 255, 255])],
                'display_color': (0, 255, 255),
                'color_space': ColorSpace.HSV
            },
            'green': {
                'ranges': [np.array([35, 70, 50]), np.array([85, 255, 255])],
                'display_color': (0, 255, 0),
                'color_space': ColorSpace.HSV
            },
            'blue': {
                'ranges': [np.array([90, 70, 50]), np.array([120, 255, 255])],
                'display_color': (255, 0, 0),
                'color_space': ColorSpace.HSV
            },
            'cyan': {
                'ranges': [np.array([85, 70, 50]), np.array([95, 255, 255])],
                'display_color': (255, 255, 0),
                'color_space': ColorSpace.HSV
            },
            'purple': {
                'ranges': [np.array([130, 70, 50]), np.array([150, 255, 255])],
                'display_color': (128, 0, 128),
                'color_space': ColorSpace.HSV
            },
            'black': {
                'ranges': [np.array([0, 0, 0]), np.array([180, 255, 50])],
                'display_color': (255, 0, 255),
                'color_space': ColorSpace.BINARY,
                'use_binary': True
            }
        }
        
        # 设置默认HLS颜色阈值配置
        self.default_hls_thresholds = {
            'red': {
                'ranges': [
                    np.array([0, 30, 80]), np.array([10, 200, 255]),
                    np.array([170, 30, 80]), np.array([179, 200, 255])
                ],
                'display_color': (0, 0, 255),
                'color_space': ColorSpace.HLS
            },
            'yellow': {
                'ranges': [np.array([25, 50, 120]), np.array([35, 200, 255])],
                'display_color': (0, 255, 255),
                'color_space': ColorSpace.HLS
            },
            'green': {
                'ranges': [np.array([40, 50, 50]), np.array([70, 200, 255])],
                'display_color': (0, 255, 0),
                'color_space': ColorSpace.HLS
            },
            'blue': {
                'ranges': [np.array([100, 50, 50]), np.array([130, 200, 255])],
                'display_color': (255, 0, 0),
                'color_space': ColorSpace.HLS
            },
            'cyan': {
                'ranges': [np.array([85, 50, 50]), np.array([95, 200, 255])],
                'display_color': (255, 255, 0),
                'color_space': ColorSpace.HLS
            },
            'purple': {
                'ranges': [np.array([130, 50, 50]), np.array([150, 200, 255])],
                'display_color': (128, 0, 128),
                'color_space': ColorSpace.HLS
            },
            'orange': {
                'ranges': [np.array([10, 50, 120]), np.array([20, 200, 255])],
                'display_color': (0, 165, 255),
                'color_space': ColorSpace.HLS
            },
            'pink': {
                'ranges': [np.array([150, 80, 100]), np.array([170, 200, 255])],
                'display_color': (203, 192, 255),
                'color_space': ColorSpace.HLS
            },
            'white': {
                'ranges': [np.array([0, 200, 0]), np.array([179, 255, 50])],
                'display_color': (255, 255, 255),
                'color_space': ColorSpace.HLS
            },
            'gray': {
                'ranges': [np.array([0, 50, 0]), np.array([179, 200, 50])],
                'display_color': (128, 128, 128),
                'color_space': ColorSpace.HLS
            },
            'black': {
                'ranges': [np.array([0, 0, 0]), np.array([179, 50, 255])],
                'display_color': (255, 0, 255),
                'color_space': ColorSpace.BINARY,
                'use_binary': True
            }
        }
        
        # 设置颜色阈值配置
        if color_thresholds is None:
            if default_color_space == ColorSpace.HLS:
                self.color_thresholds = self.default_hls_thresholds.copy()
            else:
                self.color_thresholds = self.default_hsv_thresholds.copy()
        else:
            self.color_thresholds = color_thresholds

        self.camera_matrix = np.array([
            [605.6649780273438, 0, 320.0],
            [0, 604.083984375, 240.0],
            [0, 0, 1]
        ])
    
    def set_color_space(self, color_space: ColorSpace):
        """
        设置默认色彩空间
        
        Args:
            color_space: 色彩空间类型
        """
        self.default_color_space = color_space
        
        # 根据色彩空间更新默认阈值
        if color_space == ColorSpace.HLS:
            self.color_thresholds = self.default_hls_thresholds.copy()
        else:
            self.color_thresholds = self.default_hsv_thresholds.copy()
    
    def get_color_presets(self, color_space: ColorSpace = None) -> Dict:
        """
        获取指定色彩空间的颜色预设
        
        Args:
            color_space: 色彩空间类型，None则使用默认
            
        Returns:
            颜色预设字典
        """
        if color_space is None:
            color_space = self.default_color_space
            
        if color_space == ColorSpace.HLS:
            return self.default_hls_thresholds.copy()
        else:
            return self.default_hsv_thresholds.copy()
    
    def _create_binary_mask(self, bgr_img: np.ndarray, 
                           close_iterations: int, 
                           open_iterations: int) -> np.ndarray:
        """
        使用二值化方法创建黑色掩膜
        
        Args:
            bgr_img: BGR输入图像
            close_iterations: 闭运算迭代次数
            open_iterations: 开运算迭代次数
            
        Returns:
            二值化掩膜
        """
        # 转换为灰度图
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        
        # 应用高斯模糊减少噪声
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 二值化处理：黑色区域设为255（白色），其他区域设为0（黑色）
        # 使用THRESH_BINARY_INV，使得暗区域变为白色
        _, binary_mask = cv2.threshold(blurred, self.binary_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # 形态学处理
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, self.kernel, iterations=open_iterations)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, self.kernel, iterations=close_iterations)
        
        return binary_mask
    
    def detect(self, 
               image: np.ndarray, 
               colors: Optional[Union[str, List[str]]] = None,
               color_space: Optional[ColorSpace] = None,
               close_iterations: Optional[int] = None,
               open_iterations: Optional[int] = None,
               area_threshold: Optional[int] = None,
               red_threshold: Optional[List[List[int]]] = None,
               yellow_threshold: Optional[List[List[int]]] = None,
               green_threshold: Optional[List[List[int]]] = None,
               blue_threshold: Optional[List[List[int]]] = None,
               black_threshold: Optional[List[List[int]]] = None,
               custom_thresholds: Optional[Dict[str, List[List[int]]]] = None,
               binary_threshold: Optional[int] = None) -> Union[ColorDetectionResult, List[ColorDetectionResult]]:
        """
        一行代码完成颜色检测
        
        Args:
            image: 输入BGR图像
            colors: 要检测的颜色
            color_space: 使用的色彩空间
            close_iterations: 闭运算迭代次数
            open_iterations: 开运算迭代次数
            area_threshold: 轮廓面积阈值
            red_threshold: 红色阈值
            yellow_threshold: 黄色阈值
            green_threshold: 绿色阈值
            blue_threshold: 蓝色阈值
            black_threshold: 黑色阈值
            custom_thresholds: 自定义颜色阈值字典
            binary_threshold: 二值化阈值（用于黑色检测）
            
        Returns:
            检测结果列表或单个结果
        """
        # 使用临时参数或默认参数
        temp_close_iter = close_iterations if close_iterations is not None else self.close_iterations
        temp_open_iter = open_iterations if open_iterations is not None else self.open_iterations
        temp_area_threshold = area_threshold if area_threshold is not None else self.area_threshold
        temp_binary_threshold = binary_threshold if binary_threshold is not None else self.binary_threshold
        temp_color_space = color_space if color_space is not None else self.default_color_space
        
        # 创建临时颜色阈值配置
        if temp_color_space == ColorSpace.HLS:
            temp_color_thresholds = self.default_hls_thresholds.copy()
        else:
            temp_color_thresholds = self.default_hsv_thresholds.copy()
        
        # 更新颜色阈值
        threshold_mapping = {
            'red': red_threshold,
            'yellow': yellow_threshold,
            'green': green_threshold,
            'blue': blue_threshold,
            'black': black_threshold
        }
        
        for color_name, threshold_values in threshold_mapping.items():
            if threshold_values is not None:
                temp_color_thresholds[color_name] = self._create_color_config(
                    threshold_values, color_name, temp_color_space)
        
        # 处理自定义颜色阈值
        if custom_thresholds is not None:
            for color_name, threshold_values in custom_thresholds.items():
                temp_color_thresholds[color_name] = self._create_color_config(
                    threshold_values, color_name, temp_color_space)
        
        # 确定要检测的颜色
        if colors is None:
            colors_to_detect = list(temp_color_thresholds.keys())
            return_single = False
        elif isinstance(colors, str):
            colors_to_detect = [colors]
            return_single = True
        else:
            colors_to_detect = colors
            return_single = False
        
        # 图像预处理
        processed_img = self._crop_and_resize_image(image)
        if self.enable_preprocessing:
            processed_img = self._preprocess_image(processed_img)
        
        # 只有在需要时才转换色彩空间
        converted_img = None
        height, width = processed_img.shape[:2]
        
        results = []
        
        # 处理每个颜色
        for color_name in colors_to_detect:
            if color_name not in temp_color_thresholds:
                continue
            
            color_config = temp_color_thresholds[color_name]
            
            # 检查是否使用二值化处理
            use_binary = color_config.get('use_binary', False)
            config_color_space = color_config.get('color_space', temp_color_space)
            
            if use_binary or config_color_space == ColorSpace.BINARY:
                # 使用二值化方法处理
                mask = self._create_binary_mask(processed_img, temp_close_iter, temp_open_iter)
                used_color_space = "binary"
            else:
                # 使用色彩空间方法处理
                if converted_img is None:
                    if temp_color_space == ColorSpace.HLS:
                        converted_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2HLS)
                        used_color_space = "hls"
                    else:
                        converted_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2HSV)
                        used_color_space = "hsv"
                
                color_ranges = color_config['ranges']
                mask = self._create_mask(converted_img, color_ranges, temp_close_iter, temp_open_iter)
            
            # 检测轮廓并计算位置
            detected, cx, cy, largest_contour, contour_area = self._detect_contour(mask, temp_area_threshold)
            
            # 创建结果对象
            result = ColorDetectionResult(
                color=color_name,
                detected=detected,
                center_x=cx,
                center_y=cy,
                contour_area=contour_area,
                contour=largest_contour,
                color_space=used_color_space
            )
            
            results.append(result)
        
        # 根据输入类型返回结果
        if return_single:
            return results[0] if results else ColorDetectionResult(color=colors, detected=False)
        else:
            return results
    
    def set_binary_threshold(self, threshold: int):
        """
        设置二值化阈值
        
        Args:
            threshold: 二值化阈值 (0-255)
        """
        if 0 <= threshold <= 255:
            self.binary_threshold = threshold
        else:
            raise ValueError("Binary threshold must be between 0 and 255.")
    
    def enable_black_binary_mode(self, enable: bool = True):
        """
        启用或禁用黑色的二值化模式
        
        Args:
            enable: 是否启用二值化模式
        """
        if 'black' in self.color_thresholds:
            self.color_thresholds['black']['use_binary'] = enable
    
    def draw_results(self, image: np.ndarray, results: List[ColorDetectionResult]) -> np.ndarray:
        """
        在图像上绘制检测结果
        
        Args:
            image: 输入图像
            results: 检测结果列表
            
        Returns:
            绘制后的图像
        """
        processed_img = self._crop_and_resize_image(image.copy())
        
        text_y_offset = 30
        
        for result in results:
            if not result.detected:
                continue
                
            color_name = result.color
            
            # 获取显示颜色
            display_color = (255, 255, 255)  # 默认白色
            if color_name in self.color_thresholds:
                display_color = self.color_thresholds[color_name]['display_color']
            elif color_name in self.default_hls_thresholds:
                display_color = self.default_hls_thresholds[color_name]['display_color']
            elif color_name in self.default_hsv_thresholds:
                display_color = self.default_hsv_thresholds[color_name]['display_color']
            
            # 绘制中心点
            cv2.circle(processed_img, (result.center_x, result.center_y), 10, display_color, -1)
            
            # 绘制轮廓
            if result.contour is not None:
                cv2.drawContours(processed_img, [result.contour], -1, display_color, 2)
            
            # 显示信息，包含色彩空间信息
            color_space_text = f" ({result.color_space.upper()})"
            info_text = f"{color_name}{color_space_text}: ({result.delta_x:.2f}, {result.delta_y:.2f})"
            cv2.putText(processed_img, info_text,
                       (10, text_y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, display_color, 2)
            text_y_offset += 25
        
        return processed_img
    
    def _create_color_config(self, threshold_values: List[List[int]], 
                           color_name: str, color_space: ColorSpace) -> Dict:
        """创建颜色配置"""
        ranges = []
        for i in range(0, len(threshold_values), 2):
            if i + 1 < len(threshold_values):
                ranges.append(np.array(threshold_values[i]))
                ranges.append(np.array(threshold_values[i + 1]))
        
        # 获取默认显示颜色
        default_colors = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'cyan': (255, 255, 0),
            'purple': (128, 0, 128),
            'orange': (0, 165, 255),
            'pink': (203, 192, 255),
            'white': (255, 255, 255),
            'gray': (128, 128, 128),
            'black': (255, 0, 255),
        }
        
        display_color = default_colors.get(color_name, (255, 255, 255))
        
        config = {
            'ranges': ranges,
            'display_color': display_color,
            'color_space': color_space
        }
        
        # 如果是黑色，默认启用二值化
        if color_name == 'black':
            config['use_binary'] = True
        
        return config
    
    def _preprocess_image(self, bgr_img: np.ndarray) -> np.ndarray:
        """图像预处理"""
        return cv2.bilateralFilter(bgr_img, d=5, sigmaColor=75, sigmaSpace=75)
    
    def _crop_and_resize_image(self, image: np.ndarray) -> np.ndarray:
        """裁剪并调整图像大小"""
        orig_height, orig_width = image.shape[:2]
        
        start_x = max(0, int((orig_width - self.target_width) / 2))
        start_y = max(0, int((orig_height - self.target_height) / 2))
        
        cropped = image[start_y:start_y+self.target_height, start_x:start_x+self.target_width]
        
        if cropped.shape[0] != self.target_height or cropped.shape[1] != self.target_width:
            cropped = cv2.resize(cropped, (self.target_width, self.target_height))
        
        return cropped
    
    def _create_mask(self, converted_img: np.ndarray, color_ranges: List[np.ndarray], 
                    close_iterations: int, open_iterations: int) -> np.ndarray:
        """创建颜色掩膜"""
        if len(color_ranges) == 4:
            lower1, upper1, lower2, upper2 = color_ranges
            mask1 = cv2.inRange(converted_img, lower1, upper1)
            mask2 = cv2.inRange(converted_img, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower, upper = color_ranges
            mask = cv2.inRange(converted_img, lower, upper)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=open_iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=close_iterations)
        return mask
    
    def _detect_contour(self, mask: np.ndarray, area_threshold: int) -> Tuple[bool, int, int, Optional[np.ndarray], float]:
        """轮廓检测"""
        detected = False
        cx = cy = 0
        largest_contour = None
        contour_area = 0.0
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest_contour)
            
            if contour_area > area_threshold:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    detected = True
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
        
        return detected, cx, cy, largest_contour, contour_area


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

    def set_alpha(self, alpha: float):
        """设置滤波系数"""
        if 0 < alpha < 1:
            self.alpha = alpha
        else:
            raise ValueError("Alpha must be between 0 and 1.")


class CalculateCoordinates:
    """计算坐标类"""
    
    def __init__(self, camera_matrix: np.ndarray):
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]

    def front_calculate_coordinates(self, x: float, y: float, depth: float) -> Tuple[float, float, float]:
        """计算实际坐标"""
        cx_3d = (x - self.cx) * depth / self.fx
        cy_3d = (y - self.cy) * depth / self.fy

        x_3d = depth
        y_3d = -cx_3d
        z_3d = -cy_3d
        
        return x_3d, y_3d, z_3d

    def down_calculate_coordinates(self, x: float, y: float, height: float) -> Tuple[float, float]:
        """计算实际坐标"""
        cx_3d = (x - self.cx) * height / self.fx
        cy_3d = (y - self.cy) * height / self.fy

        x_3d = -cy_3d
        y_3d = -cx_3d

        return x_3d, y_3d

    def calculate_yaw(self, delta_x: float, delta_y: float) -> float:
        """计算偏航角"""
        if delta_x == 0 and delta_y == 0:
            return 0.0
        
        # 计算偏航角，使用反正切函数
        yaw = math.atan2(delta_y, delta_x)
        
        # 将角度限制在[-π, π]范围内
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        
        return yaw