#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#define SERIAL_PORT "/dev/ch344_port0"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SerialSenderNode : public rclcpp::Node
{
public:
  SerialSenderNode() : Node("serial_sender")
  {
    // 初始化串口
    openSerialPort();

    // 创建订阅
    esp32_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/esp32", 10, std::bind(&SerialSenderNode::esp32Callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Serial sender node initialized. Listening to /esp32");
  }

  ~SerialSenderNode()
  {
    if (serial_port_ >= 0) {
      close(serial_port_);
    }
  }

private:
  void openSerialPort()
  {
    if (serial_port_ >= 0) {
      close(serial_port_);
    }

    serial_port_ = open(SERIAL_PORT, O_WRONLY | O_NOCTTY | O_SYNC);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
      return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr error: %s", strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr error: %s", strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port re-opened successfully");
    }
  }

  void esp32Callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 1) {
      try {
        sendSerialByte(0x01);
        RCLCPP_INFO(this->get_logger(), "Sent 0x01 to serial port for /esp32 = 1");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Exception: %s. Trying to reconnect serial...", e.what());
        openSerialPort();  // 尝试重新连接串口
      }
    }
  }

  void sendSerialByte(uint8_t byte)
  {
    if (serial_port_ < 0) {
      throw std::runtime_error("Serial port not open");
    }

    ssize_t bytes_written = write(serial_port_, &byte, 1);
    if (bytes_written < 0) {
      throw std::runtime_error(std::string("Write failed: ") + strerror(errno));
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr esp32_sub_;
  int serial_port_ = -1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
