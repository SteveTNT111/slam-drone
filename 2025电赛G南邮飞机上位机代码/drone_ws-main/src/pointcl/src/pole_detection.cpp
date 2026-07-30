#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "msg_tool/msg/pole.hpp"

// PCL点云处理相关头文件
#include <pcl/point_types.h>       // 点类型定义
#include <pcl/point_cloud.h>       // 点云数据结构
#include <pcl/common/common.h>     // 公共计算方法
#include <pcl/segmentation/extract_clusters.h> // 聚类分割
#include <pcl/kdtree/kdtree.h>     // KD树加速搜索
#include <pcl_conversions/pcl_conversions.h> // ROS与PCL转换

// 系统头文件
#include <mutex>                   // 互斥锁
#include <memory>                  // 智能指针
#include <cmath>                   // 数学计算
#include <algorithm>               // 算法函数
#include <vector>                  // 向量容器

/**
 * @class LowPassFilter
 * @brief 低通滤波器，用于平滑位置估计
 */
class LowPassFilter {
public:
  /**
   * @brief 构造函数
   * @param alpha 滤波系数 (0-1)，越大响应越快
   */
  LowPassFilter(double alpha = 0.7)
   : alpha_(alpha),
     initialized_(false),
     filtered_x_(0.0),
     filtered_y_(0.0) {}

  /**
   * @brief 更新滤波器
   * @param x 新的X位置
   * @param y 新的Y位置
   */
  void update(double x, double y) {
    if (!initialized_) {
      filtered_x_ = x;
      filtered_y_ = y;
      initialized_ = true;
    } else {
      // 低通滤波公式: output = α * input + (1-α) * previous_output
      filtered_x_ = alpha_ * x + (1.0 - alpha_) * filtered_x_;
      filtered_y_ = alpha_ * y + (1.0 - alpha_) * filtered_y_;
    }
  }
  
  /**
   * @brief 获取滤波后的位置
   * @param x 输出X位置
   * @param y 输出Y位置
   */
  void getPosition(double& x, double& y) const {
    if (initialized_) {
      x = filtered_x_;
      y = filtered_y_;
    } else {
      x = 0.0;
      y = 0.0;
    }
  }
  
  /**
   * @brief 获取滤波器初始化状态
   * @return 是否已初始化
   */
  bool isInitialized() const {
    return initialized_;
  }
  
  /**
   * @brief 重置滤波器状态
   */
  void reset() {
    initialized_ = false;
    filtered_x_ = 0.0;
    filtered_y_ = 0.0;
  }

private:
  double alpha_;          // 滤波系数
  bool initialized_;      // 滤波器是否已初始化
  double filtered_x_;     // 滤波后的X位置
  double filtered_y_;     // 滤波后的Y位置
};

/**
 * @class PoleDetectionNode
 * @brief 杆状物检测节点，用于从激光雷达点云中检测单根垂直杆状物体
 */
class PoleDetectionNode : public rclcpp::Node {
public:
  PoleDetectionNode() : Node("pole_detection_node") {
    // 初始化参数
    declareParameters();

    // 创建激光雷达点云订阅器
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/pointcloud2", rclcpp::QoS(10), 
      std::bind(&PoleDetectionNode::lidarCallback, this, std::placeholders::_1));
    
    // 创建杆状物检测结果发布器 - 使用单个Pole消息
    pole_publisher_ = this->create_publisher<msg_tool::msg::Pole>(
      "/detected_pole", rclcpp::QoS(10));
    
    // 创建处理定时器
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PoleDetectionNode::processPointCloud, this));
      
    // 初始化单个滤波器
    pole_filter_ = std::make_unique<LowPassFilter>(filter_alpha_);
    
    RCLCPP_INFO(this->get_logger(), "单杆检测节点已启动");
  }
  
private:
  /**
   * @brief 杆状物信息结构
   */
  struct PoleInfo {
    float x;        // X 坐标
    float y;        // Y 坐标
    float diameter; // 直径
    float distance; // 到机器人的距离
    
    // 默认构造函数
    PoleInfo() : x(0.0f), y(0.0f), diameter(0.0f), distance(0.0f) {}
    
    // 带参数的构造函数
    PoleInfo(float _x, float _y, float _d) 
      : x(_x), y(_y), diameter(_d), distance(std::sqrt(_x*_x + _y*_y)) {}
  };
  
  /**
   * @brief 声明并加载所有参数
   */
  void declareParameters() {
    // 声明聚类参数
    this->declare_parameter("cluster_tolerance", 0.1);  // 聚类距离阈值(米)
    this->declare_parameter("min_cluster_size", 1);     // 最小聚类点数
    this->declare_parameter("max_cluster_size", 10000); // 最大聚类点数
    this->declare_parameter("pole_radius_threshold", 0.3); // 杆状物半径阈值
    this->declare_parameter("min_valid_distance", 0.1);  // 最小有效距离(米)
    this->declare_parameter("min_height", 0.2);          // 最小高度要求(米)
    this->declare_parameter("pole_association_threshold", 0.5); // 杆关联距离阈值
    
    // 声明低通滤波参数
    this->declare_parameter("filter_alpha", 0.7);        // 滤波系数
    
    // 声明发布范围限制参数
    this->declare_parameter("publish_x_min", 0.1);       // X坐标最小值(米)
    this->declare_parameter("publish_x_max", 2.0);       // X坐标最大值(米)
    this->declare_parameter("publish_y_min", -1.0);      // Y坐标最小值(米)
    this->declare_parameter("publish_y_max", 1.0);       // Y坐标最大值(米)
    
    // 加载参数值
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    pole_radius_threshold_ = this->get_parameter("pole_radius_threshold").as_double();
    min_valid_distance_ = this->get_parameter("min_valid_distance").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    pole_association_threshold_ = this->get_parameter("pole_association_threshold").as_double();
    filter_alpha_ = this->get_parameter("filter_alpha").as_double();
    
    // 加载发布范围限制参数
    publish_x_min_ = this->get_parameter("publish_x_min").as_double();
    publish_x_max_ = this->get_parameter("publish_x_max").as_double();
    publish_y_min_ = this->get_parameter("publish_y_min").as_double();
    publish_y_max_ = this->get_parameter("publish_y_max").as_double();
  }

  /**
   * @brief 检查位置是否在发布范围内
   * @param x X坐标
   * @param y Y坐标
   * @return 是否在有效范围内
   */
  bool isPositionInValidRange(double x, double y) const {
    return (x >= publish_x_min_ && x <= publish_x_max_ && 
            y >= publish_y_min_ && y <= publish_y_max_);
  }

  /**
   * @brief 激光雷达点云回调函数
   * @param msg 输入的点云消息
   */
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::fromROSMsg(*msg, *cloud_);  // 转换ROS消息为PCL格式
    new_data_ = true;       // 标记有新数据需要处理
  }
  
  /**
   * @brief 点云处理主函数
   */
  void processPointCloud() {
    // 无新数据时跳过处理
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!new_data_ || cloud_->empty()) {
        return;
      }
      new_data_ = false;  // 重置数据标志
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 创建数据副本进行处理
    {
      std::lock_guard<std::mutex> lock(mutex_);
      *cloud_copy = *cloud_;
    }
    
    if (cloud_copy->empty()) {
      RCLCPP_DEBUG(this->get_logger(), "点云为空");
      return;
    }
    
    // 执行聚类分析
    std::vector<pcl::PointIndices> cluster_indices = performClustering(cloud_copy);
    
    if (cluster_indices.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "未找到有效聚类");
      return;
    }
    
    // 分析聚类结果，找到最佳杆状物
    analyzeClusters(cloud_copy, cluster_indices);
  }
  
  /**
   * @brief 执行欧几里得聚类
   * @param cloud 输入点云
   * @return 聚类索引列表
   */
  std::vector<pcl::PointIndices> performClustering(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    
    // 创建KD树加速最近邻搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    // 执行欧几里得聚类算法
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);  // 点间距阈值
    ec.setMinClusterSize(min_cluster_size_);     // 最小点数要求
    ec.setMaxClusterSize(max_cluster_size_);     // 最大点数限制
    ec.setSearchMethod(tree);                    // 设置搜索方法
    ec.setInputCloud(cloud);                     // 设置输入点云
    ec.extract(cluster_indices);                 // 执行聚类
    
    return cluster_indices;
  }
  
  /**
   * @brief 分析聚类结果，找到最佳杆状物
   * @param cloud 点云
   * @param cluster_indices 聚类索引列表
   */
  void analyzeClusters(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::vector<pcl::PointIndices>& cluster_indices) {
    
    std::vector<PoleInfo> detected_poles;
    
    // 遍历所有聚类
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      // 提取当前聚类点云
      for (const auto& idx : cluster_indices[i].indices) {
        cluster_cloud->push_back((*cloud)[idx]);
      }
      
      if (!cluster_cloud->empty()) {
        // 分析当前聚类是否为杆状物并提取特征
        PoleInfo* pole_candidate = analyzePoleCandidate(cluster_cloud);
        if (pole_candidate != nullptr) {
          // 检查杆是否在有效范围内，只有在范围内的杆才会被添加到候选列表
          if (isPositionInValidRange(pole_candidate->x, pole_candidate->y)) {
            detected_poles.push_back(*pole_candidate);
          } else {
            RCLCPP_DEBUG(this->get_logger(), 
              "检测到杆但位置超出范围，忽略: (%.2f, %.2f)", 
              pole_candidate->x, pole_candidate->y);
          }
          delete pole_candidate;
        }
      }
    }
    
    if (detected_poles.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "未找到有效范围内的杆状物");
      return; // 不发布任何消息
    }
    
    // 选择最近的杆作为检测结果
    PoleInfo best_pole = *std::min_element(detected_poles.begin(), detected_poles.end(),
      [](const PoleInfo& a, const PoleInfo& b) {
        return a.distance < b.distance;
      });
    
    // 检查是否与之前检测的杆关联
    bool is_associated = false;
    if (pole_filter_->isInitialized()) {
      double prev_x, prev_y;
      pole_filter_->getPosition(prev_x, prev_y);
      
      float distance_to_prev = std::sqrt(
        std::pow(best_pole.x - prev_x, 2) + std::pow(best_pole.y - prev_y, 2));
      
      is_associated = (distance_to_prev < pole_association_threshold_);
    }
    
    // 如果未关联且滤波器已初始化，重置滤波器
    if (!is_associated && pole_filter_->isInitialized()) {
      pole_filter_->reset();
      // RCLCPP_INFO(this->get_logger(), "检测到新杆，重置滤波器");
    }
    
    // 更新滤波器
    pole_filter_->update(best_pole.x, best_pole.y);
    
    // 获取滤波后的位置
    double filtered_x, filtered_y;
    pole_filter_->getPosition(filtered_x, filtered_y);
    
    // 发布滤波后的结果（已确保在有效范围内）
    publishPoleDetection(filtered_x, filtered_y, true);
    
    RCLCPP_DEBUG(this->get_logger(), 
      "检测到有效杆: 原始位置(%.2f, %.2f), 滤波后位置(%.2f, %.2f), 距离: %.2f米", 
      best_pole.x, best_pole.y, filtered_x, filtered_y, best_pole.distance);
  }
  
  /**
   * @brief 分析聚类是否为杆状物
   * @param cluster_cloud 聚类点云
   * @return PoleInfo指针，如果不是杆状物则返回nullptr
   */
  PoleInfo* analyzePoleCandidate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_cloud) {
    if (cluster_cloud->empty()) {
      return nullptr;
    }
    
    // 计算点云包围盒
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
    
    // 计算物理尺寸
    float width = max_pt[0] - min_pt[0];  // X轴宽度
    float depth = max_pt[1] - min_pt[1];  // Y轴深度
    float height = max_pt[2] - min_pt[2]; // Z轴高度
    
    // 计算近似直径（取最大水平尺寸）
    float diameter = std::max(width, depth);
    
    // 找到Z轴最高点作为杆的顶点
    float max_z = min_pt[2];
    float top_x = 0.0f, top_y = 0.0f;
    int top_point_count = 0;
    
    // 定义接近最高点的阈值（在最高点下方10cm内的点都认为是顶点区域）
    float top_threshold = 0.1f;
    
    // 遍历所有点，找到最高点区域的中心
    for (const auto& point : cluster_cloud->points) {
      if (point.z > max_z) {
        max_z = point.z;
      }
    }
    
    // 计算顶点区域的平均位置
    for (const auto& point : cluster_cloud->points) {
      if (point.z >= (max_z - top_threshold)) {
        top_x += point.x;
        top_y += point.y;
        top_point_count++;
      }
    }
    
    if (top_point_count == 0) {
      return nullptr;
    }
    
    // 计算顶点的平均位置
    top_x /= top_point_count;
    top_y /= top_point_count;
    
    // 计算到原点距离
    float distance = std::sqrt(top_x*top_x + top_y*top_y);
    
    // 验证杆状物特征
    bool is_pole = diameter < pole_radius_threshold_ &&     // 直径小于阈值
                   height > min_height_ &&                  // 最小高度要求
                   max_z < 2.0f &&                         // 最高点不超过2米（排除屋顶）
                   distance > min_valid_distance_;       // 最小有效距离
    
    if (is_pole) {
      // 使用顶点位置作为杆的位置
      return new PoleInfo(top_x, top_y, diameter);
    } else {
      return nullptr;
    }
  }
  
  /**
   * @brief 发布杆状物检测结果
   * @param x X坐标
   * @param y Y坐标 
   * @param detected 是否检测到杆
   */
  void publishPoleDetection(double x, double y, bool detected) {
    auto msg = std::make_unique<msg_tool::msg::Pole>();
    
    msg->x = static_cast<float>(x);
    msg->y = static_cast<float>(y);
    msg->detected = detected;
    
    // 发布消息
    pole_publisher_->publish(std::move(msg));
    
    RCLCPP_DEBUG(this->get_logger(), "发布杆检测结果: (%.2f, %.2f)", x, y);
  }
  
  // ROS2通信接口
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;  // 点云订阅器
  rclcpp::Publisher<msg_tool::msg::Pole>::SharedPtr pole_publisher_;            // 检测结果发布器
  rclcpp::TimerBase::SharedPtr timer_;                                          // 处理定时器
  
  // 数据存储
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // 当前点云
  std::mutex mutex_;          // 数据访问互斥锁
  bool new_data_ = false;      // 新数据到达标志
  
  // 杆状物跟踪
  std::unique_ptr<LowPassFilter> pole_filter_; // 单个杆的滤波器
  
  // 算法参数
  double cluster_tolerance_;   // 聚类距离阈值(米)
  int min_cluster_size_;       // 最小聚类点数
  int max_cluster_size_;       // 最大聚类点数
  double pole_radius_threshold_; // 杆状物半径阈值(米)
  double min_valid_distance_;  // 最小有效距离(米)
  double min_height_;          // 最小高度要求(米)
  double pole_association_threshold_; // 杆关联距离阈值
  double filter_alpha_;        // 低通滤波系数
  
  // 发布范围限制参数
  double publish_x_min_;       // X坐标最小值(米)
  double publish_x_max_;       // X坐标最大值(米)
  double publish_y_min_;       // Y坐标最小值(米)
  double publish_y_max_;       // Y坐标最大值(米)
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  // 初始化ROS2
  auto node = std::make_shared<PoleDetectionNode>();  // 创建节点实例
  rclcpp::spin(node);         // 运行节点
  rclcpp::shutdown();         // 关闭节点
  return 0;
}