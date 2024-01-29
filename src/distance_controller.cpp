// export RCUTILS_CONSOLE_OUTPUT_FORMAT="[${severity}] [${time}] [${name}]: ${message}"
// general libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <chrono>
#include <cstdlib>

class DistanceController : public rclcpp::Node
{
public:
    DistanceController()
        : Node("distance_controller_node"), MAX_LINEAR_SPEED_(0.8), MAX_ANGULAR_SPEED_(3.14)
    {
        // create a subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 
            10, std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));
        // create a publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    // subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    float current_x_;
    float current_y_;
    float current_theta_;
    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    const float MAX_LINEAR_SPEED_;
    const float MAX_ANGULAR_SPEED_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get the current distance from the odometry message
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = quat2rpy(msg->pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(), "Odom values: x: '%.2f', y: '%.2f', theta: '%.2f'", current_x_, current_y_, current_theta_);
    }

    double quat2rpy(const geometry_msgs::msg::Quaternion& quat)
    {
        // Convert the quaternion to a tf2 Quaternion
        tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);

        // Convert the quaternion to a rotation matrix
        tf2::Matrix3x3 mat(tf2_quat);

        // Get the roll, pitch, and yaw from the rotation matrix
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Return the yaw
        return yaw;
    }

    template <typename T>
    T saturate(T var, T min, T max)
    {
        if (var > max)
        {
            return max;
        }
        else if (var < min)
        {
            return min;
        }
        else
        {
            return var;
        }
    }

    void robot_move(float linear_speed, float angular_speed)
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_speed;
        cmd_vel_msg.angular.z = angular_speed;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }


};

int main(int argc, char *argv[])
{
    setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}]: [{message}]", 1);
    rclcpp::init(argc, argv);
    DistanceController distance_controller;
    rclcpp::spin( std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}