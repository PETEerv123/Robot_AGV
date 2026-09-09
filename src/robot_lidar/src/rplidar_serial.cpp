#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class RplidarNode : public rclcpp::Node {
public:
  RplidarNode() : Node("robot_lidar") {
    // 1. Khai báo Parameters
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_frame");

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // 2. Khởi tạo Lidar Driver
    auto drv_res = sl::createLidarDriver();
    if (!drv_res) {
      RCLCPP_ERROR(get_logger(), "Không thể tạo Driver RPLIDAR!");
      return;
    }
    lidar_ = *drv_res;

    // 3. Chỉ bật Timer nếu kết nối thành công
    if (connect()) {
      timer_ = create_wall_timer(100ms, std::bind(&RplidarNode::scan, this));
    }
  }

  ~RplidarNode() {
    if (lidar_) {
      lidar_->stop();
      lidar_->disconnect();
      delete lidar_;
      lidar_ = nullptr;
    }
    if (channel_) {
      delete channel_;
      channel_ = nullptr;
    }
  }

private:
  bool connect() {
    auto channel_res = sl::createSerialPortChannel(port_, baudrate_);
    if (!channel_res) {
      RCLCPP_ERROR(get_logger(), "Không thể tạo Serial Channel trên cổng %s",
                   port_.c_str());
      return false;
    }
    channel_ = *channel_res;

    auto result = lidar_->connect(channel_);
    if (!SL_IS_OK(result)) {
      RCLCPP_ERROR(get_logger(), "Lỗi kết nối RPLIDAR tại %s (Baud: %d)",
                   port_.c_str(), baudrate_);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Đã kết nối RPLIDAR thành công tại %s",
                port_.c_str());

    // Bắt đầu quét Lidar
    lidar_->startScan(false, true);
    return true;
  }

  void scan() {
    if (!lidar_ || !lidar_->isConnected())
      return;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = 8192;

    // Đọc dữ liệu quét (thêm timeout 1000ms để tránh treo)
    auto result = lidar_->grabScanDataHq(nodes, count, 1000);
    if (!SL_IS_OK(result) || count == 0) {
      return;
    }

    lidar_->ascendScanData(nodes, count);

    auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();

    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = frame_id_;
    constexpr int NUM_SCAN = 720;

    msg->angle_min = 0;
    msg->angle_max = 2.0 * M_PI;
    msg->angle_increment = (msg->angle_max - msg->angle_min) / (NUM_SCAN - 1);

    msg->range_min = 0.20;
    msg->range_max = 12.0;

    msg->ranges.assign(NUM_SCAN, std::numeric_limits<float>::infinity());
    msg->intensities.assign(NUM_SCAN, 0.0f);

    for (size_t i = 0; i < count; i++) {
      float distance = (nodes[i].dist_mm_q2 / 4.0f) / 1000.0f;

      if (distance < msg->range_min || distance > msg->range_max)
        continue;

      float angle_deg = 360.0f - (nodes[i].angle_z_q14 * 90.0f / 16384.0f);

      float angle = angle_deg * M_PI / 180.0f;
      while (angle >= 2.0f * M_PI)
        angle -= 2.0f * M_PI;

      // while (angle <= -2.0f * M_PI)
      //   angle += 2.0f * M_PI;

      // while (angle < -M_PI)
      //   angle += 2.0f * M_PI;

      int index = static_cast<int>(angle / msg->angle_increment);

      if (index >= 0 && index < NUM_SCAN) {
        msg->ranges[index] = distance;
        msg->intensities[index] = static_cast<float>(nodes[i].quality >> 2);
      }
    }

    scan_pub_->publish(std::move(msg));
  }

private:
  std::string port_;
  int baudrate_;
  std::string frame_id_;

  sl::ILidarDriver *lidar_{nullptr};
  sl::IChannel *channel_{nullptr};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RplidarNode>());
  rclcpp::shutdown();
  return 0;
}
