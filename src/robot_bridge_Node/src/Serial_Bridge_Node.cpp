#include <chrono>
#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <serial/serial.h>

using namespace std::chrono_literals;

class SerialBridge : public rclcpp::Node
{
public:
    SerialBridge() : Node("serial_bridge")
    {
        serial_.setPort("/dev/ttyUSB0");
        serial_.setBaudrate(115200);
        serial_.open();

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",  10, std::bind(&SerialBridge::cmdVelCallback,this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10);

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>( "/imu/data", 10);
        serial_timer_ =create_wall_timer(10ms,  std::bind(&SerialBridge::readSerial, this));

        watchdog_timer_ = create_wall_timer(100ms,std::bind(&SerialBridge::watchdog, this));
        last_cmd_time_ = now();
    }

private:

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::stringstream ss;

        ss << msg->linear.x << " "
           << msg->linear.y << " "
           << msg->angular.z << "\n";

        serial_.write(ss.str());

        last_cmd_time_ = now();
    }

    //-----------------------------------------
    // Stop if timeout
    //-----------------------------------------

    void watchdog()
    {
        double dt =  (now() - last_cmd_time_).seconds();

        if(dt > 0.5)
        {
            serial_.write("0 0 0\n");
        }
    }


    void readSerial()
    {
        if(!serial_.available())
            return;

        std::string line =
            serial_.readline(100, "\n");

        std::stringstream ss(line);

        double vx, vy, wz, yaw;

        ss >> vx >> vy >> wz >> yaw;

        nav_msgs::msg::Odometry odom;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        odom_pub_->publish(odom);

        sensor_msgs::msg::Imu imu;

        imu.orientation.z = yaw;

        imu_pub_->publish(imu);
    }

    //-----------------------------------------

    serial::Serial serial_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr serial_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_cmd_time_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<SerialBridge>());

    rclcpp::shutdown();
}