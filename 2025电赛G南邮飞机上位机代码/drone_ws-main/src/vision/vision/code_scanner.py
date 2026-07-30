import cv2
import numpy as np
from pyzbar import pyzbar


class CodeScanner:
    """
    统一的条码和二维码扫描器
    支持条形码和二维码的检测与解码
    """
    
    def __init__(self, 
                 zoom_factor=2.0, 
                 use_wechat_detector=True):
        """
        初始化扫描器
        
        Args:
            zoom_factor (float): 条形码扫描时的放大倍数
            use_wechat_detector (bool): 是否使用微信二维码检测器
        """
        self.zoom_factor = zoom_factor
        self.use_wechat_detector = use_wechat_detector
        
        # 初始化微信二维码检测器
        if use_wechat_detector:
            try:
                self.qr_detector = cv2.wechat_qrcode.WeChatQRCode(
                    "/home/kevin/opencv_3rdparty/detect.prototxt",
                    "/home/kevin/opencv_3rdparty/detect.caffemodel", 
                    "/home/kevin/opencv_3rdparty/sr.prototxt",
                    "/home/kevin/opencv_3rdparty/sr.caffemodel"
                )
                print("微信二维码检测器初始化成功")
            except Exception as e:
                print(f"微信二维码检测器初始化失败: {e}")
                self.use_wechat_detector = False
                self.qr_detector = None
    
    def _crop_and_zoom_image(self, image, center_x, center_y):
        """
        以指定中心点为中心，裁剪并放大图像
        """
        try:
            h, w = image.shape[:2]
            
            # 计算放大后的裁剪区域大小
            crop_w = int(w / self.zoom_factor)
            crop_h = int(h / self.zoom_factor)
            
            # 计算裁剪区域的左上角坐标
            x1 = max(0, center_x - crop_w // 2)
            y1 = max(0, center_y - crop_h // 2)
            x2 = min(w, x1 + crop_w)
            y2 = min(h, y1 + crop_h)
            
            # 调整裁剪区域以确保完整
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
            
            # 裁剪图像
            cropped = image[y1:y2, x1:x2]
            
            # 放大到原始尺寸
            zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_CUBIC)
            
            return zoomed, (x1, y1, x2, y2)
        
        except Exception as e:
            print(f'图像裁剪放大时出错: {str(e)}')
            return image, None
    
    def _preprocess_for_barcode(self, image):
        """
        条形码扫描预处理
        """
        try:
            # 转换为灰度图
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image.copy()
            
            return gray
        
        except Exception as e:
            print(f'条形码预处理时出错: {str(e)}')
            return image
    
    def _scan_barcodes(self, image, center_x=None, center_y=None):
        """
        扫描条形码
        """
        results = []
        
        try:
            # 如果没有指定中心点，使用图像中心
            if center_x is None:
                center_x = image.shape[1] // 2
            if center_y is None:
                center_y = image.shape[0] // 2
            
            # 裁剪并放大图像
            zoomed_image, crop_info = self._crop_and_zoom_image(image, center_x, center_y)
            
            # 预处理图像
            processed_image = self._preprocess_for_barcode(zoomed_image)
            
            # 扫描条形码
            barcodes = pyzbar.decode(processed_image)
            
            for barcode in barcodes:
                barcode_data = barcode.data.decode('utf-8')
                barcode_type = barcode.type
                (x, y, w, h) = barcode.rect
                
                result = {
                    'type': 'barcode',
                    'format': barcode_type,
                    'data': barcode_data,
                    'position': (x, y, w, h),
                    'center_x': center_x,
                    'center_y': center_y
                }
                
                results.append(result)
        
        except Exception as e:
            print(f'条形码扫描时出错: {str(e)}')
        
        return results
    
    def _scan_qrcodes(self, image):
        """
        扫描二维码
        """
        results = []
        
        try:
            if self.use_wechat_detector and self.qr_detector:
                # 使用微信检测器
                res, points = self.qr_detector.detectAndDecode(image)
                
                if res:
                    for i, qr_data in enumerate(res):
                        if qr_data:  # 确保数据不为空
                            result = {
                                'type': 'qrcode',
                                'format': 'QR_CODE',
                                'data': qr_data,
                                'points': points[i] if i < len(points) else None
                            }
                            results.append(result)
            else:
                # 使用pyzbar检测二维码
                qr_codes = pyzbar.decode(image)
                for qr in qr_codes:
                    if qr.type == 'QRCODE':
                        qr_data = qr.data.decode('utf-8')
                        (x, y, w, h) = qr.rect
                        
                        result = {
                            'type': 'qrcode',
                            'format': 'QR_CODE',
                            'data': qr_data,
                            'position': (x, y, w, h)
                        }
                        results.append(result)
        
        except Exception as e:
            print(f'二维码扫描时出错: {str(e)}')
        
        return results
    
    def scan(self, frame, scan_barcode=True, scan_qrcode=True, center_x=None, center_y=None):
        """
        扫描frame中的条码和二维码
        
        Args:
            frame: 输入的图像帧
            scan_barcode (bool): 是否扫描条形码
            scan_qrcode (bool): 是否扫描二维码
            center_x (int): 条形码扫描的中心X坐标（可选）
            center_y (int): 条形码扫描的中心Y坐标（可选）
        
        Returns:
            list: 扫描结果列表，每个元素包含：
                {
                    'type': 'barcode' 或 'qrcode',
                    'format': 码的格式,
                    'data': 解码的数据,
                    'position': 位置信息（条形码）或 'points': 顶点坐标（二维码）
                }
        """
        results = []
        
        if frame is None:
            print("输入的frame为空")
            return results
        
        # 扫描条形码
        if scan_barcode:
            barcode_results = self._scan_barcodes(frame, center_x, center_y)
            results.extend(barcode_results)
        
        # 扫描二维码
        if scan_qrcode:
            qrcode_results = self._scan_qrcodes(frame)
            results.extend(qrcode_results)
        
        return results
    
    def draw_results(self, frame, results):
        """
        在图像上绘制扫描结果
        
        Args:
            frame: 原始图像
            results: scan()方法返回的结果列表
        
        Returns:
            绘制了结果的图像
        """
        draw_frame = frame.copy()
        
        for result in results:
            if result['type'] == 'barcode':
                # 绘制条形码
                if 'position' in result:
                    x, y, w, h = result['position']
                    cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(draw_frame, f"{result['format']}: {result['data']}", 
                              (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            elif result['type'] == 'qrcode':
                # 绘制二维码
                if 'points' in result and result['points'] is not None:
                    # 使用微信检测器的点坐标
                    points = result['points'].astype(int)
                    cv2.polylines(draw_frame, [points], True, (0, 255, 0), 2)
                elif 'position' in result:
                    # 使用pyzbar的矩形坐标
                    x, y, w, h = result['position']
                    cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # 添加文本标签
                text = f"QR: {result['data']}"
                cv2.putText(draw_frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return draw_frame
