#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class DistanceController : public rclcpp::Node
{
public:
    DistanceController()
        : Node("distance_controller")
    {
        // create a subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/rosbot_xl_base_controller/odom", 
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
        pub_msg_.linear.x = linear_speed;
        pub_msg_.angular.z = angular_speed;
        publisher_->publish(pub_msg_);
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    DistanceController distance_controller;
    rclcpp::spin( std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}