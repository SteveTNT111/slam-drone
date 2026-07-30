#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading

class VisionTaskManager:
    """全局任务管理器"""
    
    def __init__(self):
        self.current_task = 1  # 默认任务1
        self.lock = threading.Lock()
        self._subscribers = []
    
    def set_task(self, task_id):
        """设置当前任务"""
        with self.lock:
            if task_id in [1, 2]:
                self.current_task = task_id
                print(f"任务切换到: {task_id}")
                return True
            return False
    
    def get_task(self):
        """获取当前任务"""
        with self.lock:
            return self.current_task
    
    def is_task_active(self, task_id):
        """检查指定任务是否激活"""
        return self.get_task() == task_id
    
    def add_subscriber(self, callback):
        """添加任务变更订阅者"""
        self._subscribers.append(callback)
    
    def notify_subscribers(self, task_id):
        """通知所有订阅者任务变更"""
        for callback in self._subscribers:
            try:
                callback(task_id)
            except Exception as e:
                print(f"通知订阅者失败: {e}")

# 全局任务管理器实例
vision_task_manager = VisionTaskManager()


class VisionTaskSubscriberNode(Node):
    """任务订阅节点，负责监听/task话题"""
    
    def __init__(self):
        super().__init__('vision_task_subscriber')
        
        # 订阅任务切换话题
        self.vision_task_subscriber = self.create_subscription(
            Int32,
            '/task',
            self.task_callback,
            10
        )
        
        self.get_logger().info('任务订阅节点已启动，监听 /task 话题')
        self.get_logger().info('发布消息切换任务: 1=圆环检测, 2=杆检测')
    
    def task_callback(self, msg):
        """处理任务切换消息"""
        task_id = msg.data
        self.get_logger().info(f'接收到任务切换请求: {task_id}')
        
        if vision_task_manager.set_task(task_id):
            vision_task_manager.notify_subscribers(task_id)
            self.get_logger().info(f'任务已切换到: {task_id}')
        else:
            self.get_logger().warn(f'无效的任务ID: {task_id}，支持的任务: 1, 2')

# 全局任务订阅节点实例
_vision_task_subscriber_node = None

def init_vision_task_manager():
    """初始化任务管理器（在主函数中调用）"""
    global _vision_task_subscriber_node
    if _vision_task_subscriber_node is None:
        _vision_task_subscriber_node = VisionTaskSubscriberNode()
    return _vision_task_subscriber_node

def get_vision_task_manager():
    """获取全局任务管理器"""
    return vision_task_manager
