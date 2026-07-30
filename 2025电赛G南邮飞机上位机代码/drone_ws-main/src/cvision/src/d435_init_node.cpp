#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

class D435InitNode : public rclcpp::Node {
public:
    D435InitNode() : Node("d435_init_node") {
        // 订阅RGB和深度图像话题
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 1,
            std::bind(&D435InitNode::rgb_callback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/aligned_depth_to_color/image_raw", 1,
            std::bind(&D435InitNode::depth_callback, this, std::placeholders::_1));

        // 发布处理后的RGB和深度图像
        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/d435/rgb/image_raw", 1);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/d435/camera/depth/image_raw", 1);
    }

private:
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat resized_img;
            
            // 调整大小为640×480
            // cv::resize(cv_ptr->image, resized_img, cv::Size(640, 480));
            
            // 创建新的图像消息
            auto resized_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
            
            // 发布调整大小后的图像
            rgb_pub_->publish(*resized_msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat resized_img;
            
            // 调整大小为640×480
            // cv::resize(cv_ptr->image, resized_img, cv::Size(640, 480));
            
            // 创建新的图像消息
            auto resized_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_16UC1, cv_ptr->image).toImageMsg();
            
            // 发布调整大小后的图像
            depth_pub_->publish(*resized_msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D435InitNode>());
    rclcpp::shutdown();
    return 0;
}