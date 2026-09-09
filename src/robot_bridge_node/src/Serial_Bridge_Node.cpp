#include <chrono>
#include <cmath>
#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
      serial_.flushInput();
      RCLCPP_INFO(get_logger(), "Kết nối port Mega: %s", port.c_str());

      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10);
      nav2_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10,
          std::bind(&SerialBridge::Send_cmd_vel, this, std::placeholders::_1));
      // goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      //     "/goal_pose", 10,
      //     std::bind(&SerialBridge::GoalCallback, this,
      //     std::placeholders::_1));
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
        if (msg.length() < 10) {
          continue;
        }
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
        mega_x_ = f_x;
        mega_y_ = f_y;
        odom_.header.stamp = this->get_clock()->now();
        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_footprint";

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
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;

    // if (std::abs(wz) > 0.001 && std::abs(wz) < 0.3) {
    //   wz = std::copysign(0.5, wz);
    // }

    std::stringstream ss;
    std::stringstream debug_ss;
    ss << "nav2," << vx << "," << vy << "," << wz << "\n";
    debug_ss << "Vx: " << vx << ",Vy: " << vy << ",Wz: " << wz;
    RCLCPP_INFO(get_logger(), "%s", debug_ss.str().c_str());
    if (serial_.isOpen()) {
      serial_.write(ss.str());
    }
  }
  void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double goal_x_ = msg->pose.position.x;
    double goal_y_ = msg->pose.position.y;

    RCLCPP_INFO(get_logger(), "Goal: X=%.3f Y=%.3f", goal_x_, goal_y_);

    double dx = goal_x_ - mega_x_;
    double dy = goal_y_ - mega_y_;

    double error = std::sqrt(dx * dx + dy * dy);

    RCLCPP_INFO(get_logger(), "Mega: X=%.3f Y=%.3f | Error=%.3f m", mega_x_,
                mega_y_, error);

    if (error <= 3.0) {
      // RCLCPP_INFO(get_logger(), "Error <= 0.5 m -> khong dieu chinh");
      return;
    }

    RCLCPP_WARN(get_logger(), "Error > 0.5 m -> dieu chinh Mega");

    std::stringstream ss;

    ss << "spos," << goal_x_ << "," << goal_y_ << "\n";

    if (serial_.isOpen()) {
      serial_.write(ss.str());
    }
  }
  /* private variable */
  double mega_x_ = 0.0;
  double mega_y_ = 0.0;
  /* shared Ptr */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
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
