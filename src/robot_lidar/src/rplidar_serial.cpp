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

    // Sắp xếp góc quét từ 0 đến 360 độ
    lidar_->ascendScanData(nodes, count);

    auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();

    msg->header.stamp = now();
    msg->header.frame_id = frame_id_;

    msg->angle_min = 0.0;
    msg->angle_max = 2.0 * M_PI;
    msg->angle_increment = (msg->angle_max - msg->angle_min) / count;

    msg->range_min = 0.15; // 15 cm
    msg->range_max = 12.0; // 12 m

    msg->ranges.resize(count);
    msg->intensities.resize(count);

    for (size_t i = 0; i < count; i++) {
      // Đổi từ định dạng Q2 (dist_mm_q2 / 4.0) sang đơn vị Mét
      float distance_m = (nodes[i].dist_mm_q2 / 4.0f) / 1000.0f;

      // XỬ LÝ LỖI ĐIỂM 0 / NGOÀI KHOẢNG ĐỌC CHO SLAM & NAV2
      if (distance_m < msg->range_min || distance_m > msg->range_max) {
        msg->ranges[i] = std::numeric_limits<float>::infinity();
      } else {
        msg->ranges[i] = distance_m;
      }

      // Cường độ phản xạ tín hiệu (Quality)
      msg->intensities[i] = static_cast<float>(nodes[i].quality >> 2);
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
