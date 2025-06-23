#include <iostream>
#include <memory>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

class Safety : public rclcpp::Node {
public:
    Safety() : Node("safety_node") {
        // Initialize publishers
        brake_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        brake_bool_pub_ = create_publisher<std_msgs::msg::Bool>("/brake_bool", 10);

        // Initialize subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&Safety::drive_callback, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Safety::scan_callback, this, std::placeholders::_1));
    }

private:
    double speed_ = 0.0;
    const double TTC_THRESHOLD_ = 0.4;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr brake_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_bool_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        // Update current speed from odometry
        speed_ = msg->twist.twist.linear.x;
        RCLCPP_INFO(this->get_logger(), "Current speed: %.2f m/s", speed_);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        RCLCPP_INFO(this->get_logger(), "Received laser scan message");
        double min_ttc = std::numeric_limits<double>::max();

        // Calculate Time-to-Collision (TTC) for each valid laser beam
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            const float range = scan_msg->ranges[i];
            
            if (std::isinf(range) || std::isnan(range)) {
                continue; // Skip invalid ranges
            }

            const float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            const float proj_velocity = speed_ * std::cos(angle);

            if (proj_velocity <= 0) {
                continue; // Only consider closing velocities
            }

            const float ttc = range / proj_velocity;
            if (ttc < min_ttc) {
                min_ttc = ttc;
            }
        }

        // Prepare messages
        auto brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
        auto bool_msg = std_msgs::msg::Bool();

        if (min_ttc <= TTC_THRESHOLD_) {
            RCLCPP_WARN(this->get_logger(), "Emergency brake triggered! TTC: %.2f seconds", min_ttc);
            bool_msg.data = true;
            brake_msg.drive.speed = 0.0; // Stop the vehicle
            brake_pub_->publish(brake_msg);
        } else {
            bool_msg.data = false;
        }

        // Publish boolean status regardless of braking condition
        brake_bool_pub_->publish(bool_msg);
    }
};

int main(int argc, char **argv) {
    std::cout << "Safety node running" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}