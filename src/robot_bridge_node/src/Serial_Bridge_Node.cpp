#include <chrono>
#include <exception>
#include <memory>
#include <sstream>

// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>

using namespace std::chrono_literals;
std::string port = "/dev/ttyUSB0";
class SerialBridge : public rclcpp::Node {
public:
  SerialBridge(const std::string &port, int baud) : Node("Serial_Bridge") {
    try {
      serial_.setPort(port);
      serial_.setBaudrate(baud);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      serial_.setTimeout(timeout);
      serial_.open();

      timer_ =
          create_wall_timer(10ms, std::bind(&SerialBridge::readserial, this));
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
  }

private:
  void readserial() {
    if (serial_.available()) {
      std::string msg = serial_.readline();
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    }
  }
  /*shared Ptr*/
  rclcpp::TimerBase::SharedPtr timer_;
  /*library class*/
  serial::Serial serial_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridge>("/dev/ttyACM0", 115200));
  rclcpp::shutdown();
}
