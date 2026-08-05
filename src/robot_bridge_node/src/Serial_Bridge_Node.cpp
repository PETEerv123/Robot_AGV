#include <chrono>
#include <cmath>
#include <exception>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace std::chrono_literals;

std::string port = "/dev/ttyUSB0";

class SerialBridge : public rclcpp::Node {
public:
  SerialBridge(const std::string &port, int baud) : Node("serial_bridge") {
    try {
      serial_.setPort(port);
      serial_.setBaudrate(baud);

      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      serial_.setTimeout(timeout);

      serial_.open();

      if (!serial_.isOpen()) {
        RCLCPP_ERROR(get_logger(), "Cannot open serial port.");
        return;
      }

      RCLCPP_INFO(get_logger(), "Kết nối port Mega: %s", port.c_str());

      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10);
      nav2_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10,
          std::bind(&SerialBridge::Send_cmd_vel, this, std::placeholders::_1));
      timer_ =
          create_wall_timer(10ms, std::bind(&SerialBridge::readserial, this));

    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
  }

private:
  void readserial() {
    while (serial_.available()) {
      try {

        std::string msg = serial_.readline();

        std::stringstream ss(msg);

        std::string type;
        std::string vx;
        std::string vy;
        std::string wz;
        std::string yaw;
        std::string x;
        std::string y;
        std::getline(ss, type, ',');
        std::getline(ss, vx, ',');
        std::getline(ss, vy, ',');
        std::getline(ss, wz, ',');
        std::getline(ss, yaw, ',');
        std::getline(ss, x, ',');
        std::getline(ss, y, ',');

        if (type != "odom_raw") {
          continue;
        }

        float f_vx = std::stof(vx);
        float f_vy = std::stof(vy);
        float f_wz = std::stof(wz);
        float f_yaw = std::stof(yaw);
        float f_x = std::stof(x);
        float f_y = std::stof(y);

        odom_.header.stamp = this->get_clock()->now();
        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";

        odom_.pose.pose.position.x = f_x;
        odom_.pose.pose.position.y = f_y;
        odom_.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, f_yaw);
        odom_.pose.pose.orientation = tf2::toMsg(q);

        odom_.twist.twist.linear.x = f_vx;
        odom_.twist.twist.linear.y = f_vy;
        odom_.twist.twist.linear.z = 0.0;

        odom_.twist.twist.angular.x = 0.0;
        odom_.twist.twist.angular.y = 0.0;
        odom_.twist.twist.angular.z = f_wz;

        odom_pub_->publish(odom_);

        // RCLCPP_INFO(get_logger(),
        //             "x=%.3f  y=%.3f  yaw=%.3f  vx=%.3f  vy=%.3f  wz=%.3f",
        //             f_x, f_y, f_yaw, f_vx, f_vy, f_wz);
        //
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "%s", e.what());
      }
    }
  }

  void Send_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::stringstream ss;
    ss << "nav2," << msg->linear.x << "," << msg->linear.y << ","
       << msg->angular.z << "\n";
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    if (serial_.isOpen()) {
      serial_.write(ss.str());
    }
  }

  /* private variable */

  /* shared Ptr */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
  /* library class */
  serial::Serial serial_;
  nav_msgs::msg::Odometry odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridge>("/dev/ttyACM0", 115200));
  rclcpp::shutdown();
  return 0;
}
