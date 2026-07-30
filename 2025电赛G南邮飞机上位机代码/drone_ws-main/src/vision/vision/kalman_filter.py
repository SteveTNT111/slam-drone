import numpy as np

class KalmanFilter:
    """
    卡尔曼滤波器实现，适用于线性动态系统
    
    属性:
        dim_state (int): 状态向量维度
        dim_measurement (int): 测量向量维度
        A (ndarray): 状态转移矩阵
        B (ndarray): 控制输入矩阵
        H (ndarray): 测量矩阵
        Q (ndarray): 过程噪声协方差
        R (ndarray): 测量噪声协方差
        P (ndarray): 估计误差协方差
        x (ndarray): 状态向量
    """
    
    def __init__(self, dim_state=4, dim_measurement=2, dt=1.0):
        """
        初始化卡尔曼滤波器
        
        参数:
            dim_state (int): 状态向量维度 (默认为4: x, y, vx, vy)
            dim_measurement (int): 测量向量维度 (默认为2: x, y)
            dt (float): 时间步长 (默认为1.0)
        """
        self.dim_state = dim_state
        self.dim_measurement = dim_measurement
        self.dt = dt
        
        # 初始化状态向量和协方差矩阵
        self.x = np.zeros((dim_state, 1))
        self.P = np.eye(dim_state) * 100  # 初始不确定性较高
        
        # 设置默认模型矩阵
        self._setup_default_matrices()
        
        # 内部标志，用于跟踪滤波器是否已收到第一次测量
        self._initialized = False
    
    def _setup_default_matrices(self):
        """
        设置默认的系统模型矩阵
        默认模型: 匀速直线运动模型，状态为[x, y, vx, vy]
        """
        # 状态转移矩阵 (匀速运动模型)
        self.A = np.eye(self.dim_state)
        if self.dim_state == 4:  # 如果是位置+速度模型
            self.A[0, 2] = self.dt  # x += vx * dt
            self.A[1, 3] = self.dt  # y += vy * dt
        
        # 测量矩阵 (默认只测量位置)
        self.H = np.zeros((self.dim_measurement, self.dim_state))
        for i in range(self.dim_measurement):
            self.H[i, i] = 1.0
            
        # 控制输入矩阵 (默认无控制输入)
        self.B = np.zeros((self.dim_state, 1))
        
        # 过程噪声协方差 (系统模型的不确定性)
        self.Q = np.eye(self.dim_state) * 0.01
        if self.dim_state == 4:  # 位置+速度模型的过程噪声
            # 位置误差较小，速度误差较大
            self.Q[0, 0] = 0.01  # x位置噪声
            self.Q[1, 1] = 0.01  # y位置噪声
            self.Q[2, 2] = 0.05   # x速度噪声
            self.Q[3, 3] = 0.05   # y速度噪声
        
        # 测量噪声协方差 (传感器噪声)
        self.R = np.eye(self.dim_measurement) * 0.1
    
    def predict(self, u=None):
        """
        执行卡尔曼滤波预测步骤
        
        参数:
            u (ndarray, 可选): 控制向量
            
        返回:
            ndarray: 预测后的状态估计
        """
        # 如果滤波器尚未初始化，跳过预测
        if not self._initialized:
            return self.x
        
        # 如果没有控制输入，使用零向量
        if u is None:
            u = np.zeros((1, 1))
        
        # 状态预测: x = Ax + Bu
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        
        # 误差协方差预测: P = APA^T + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        
        return self.x
    
    def update(self, measurement):
        """
        执行卡尔曼滤波更新步骤
        
        参数:
            measurement (ndarray): 测量值向量
            
        返回:
            ndarray: 更新后的状态估计
        """
        # 将测量值转换为正确的形状
        z = np.array(measurement).reshape(self.dim_measurement, 1)
        
        # 如果是第一次测量，直接初始化状态
        if not self._initialized:
            for i in range(min(self.dim_state, self.dim_measurement)):
                self.x[i, 0] = z[i, 0]
            
            # 如果状态维度大于测量维度，保持默认值
            self._initialized = True
            return self.x
        
        # 计算测量残差: y = z - Hx
        y = z - np.dot(self.H, self.x)
        
        # 计算残差协方差: S = HPH^T + R
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        
        # 计算卡尔曼增益: K = PH^T * S^(-1)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # 更新状态估计: x = x + Ky
        self.x = self.x + np.dot(K, y)
        
        # 更新估计误差协方差: P = (I - KH)P
        I = np.eye(self.dim_state)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        
        return self.x
    
    def get_state(self):
        """
        获取当前状态估计
        
        返回:
            ndarray: 当前状态向量
        """
        return self.x
    
    def reset(self):
        """重置滤波器状态"""
        self.x = np.zeros((self.dim_state, 1))
        self.P = np.eye(self.dim_state) * 100
        self._initialized = False
    
    def configure(self, process_noise=None, measurement_noise=None, 
                 initial_state=None, initial_covariance=None):
        """
        配置滤波器参数
        
        参数:
            process_noise (ndarray, 可选): 过程噪声协方差矩阵
            measurement_noise (ndarray, 可选): 测量噪声协方差矩阵
            initial_state (ndarray, 可选): 初始状态向量
            initial_covariance (ndarray, 可选): 初始协方差矩阵
        """
        if process_noise is not None:
            self.Q = process_noise
            
        if measurement_noise is not None:
            self.R = measurement_noise
            
        if initial_state is not None:
            self.x = initial_state
            self._initialized = True
            
        if initial_covariance is not None:
            self.P = initial_covariance
    
    def set_dt(self, dt):
        """
        更新时间步长并重新计算状态转移矩阵
        
        参数:
            dt (float): 新的时间步长
        """
        self.dt = dt
        # 更新状态转移矩阵中的时间相关部分
        if self.dim_state == 4:  # 位置+速度模型
            self.A[0, 2] = self.dt  # x += vx * dt
            self.A[1, 3] = self.dt  # y += vy * dt
