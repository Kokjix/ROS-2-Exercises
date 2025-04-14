#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class OdomPublisher : public rclcpp::Node
{
    private:
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr vel_subscriber_;
        sensor_msgs::msg::Imu::SharedPtr imu_msg;
        nav_msgs::msg::Odometry odom_msg;
        float x_, y_;
        float wheel_radius;
        double yaw_;
        double latest_velocity_;
        rclcpp::Time last_time_;
    public:
        OdomPublisher();
        void imu_subscriber_callback(std::shared_ptr<sensor_msgs::msg::Imu> msg);
        void vel_subscriber_callback(std::shared_ptr<sensor_msgs::msg::JointState> msg);
        void publisher_control_loop();
        ~OdomPublisher();
};

OdomPublisher::OdomPublisher() : Node("odom_publisher"), x_(0.0), y_(0.0), yaw_(0.0), count_(0)
{
    wheel_radius = 0.01;
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/robot/imu/data", 
        10, std::bind(&OdomPublisher::imu_subscriber_callback, this, std::placeholders::_1));

    vel_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/robot/robotnik_base_controller/wheels_commands_output",
        10, std::bind(&OdomPublisher::vel_subscriber_callback, this, std::placeholders::_1)
    );
    
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odom", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&OdomPublisher::publisher_control_loop, this));
    last_time_ = this->now();
}

auto OdomPublisher::imu_subscriber_callback(std::shared_ptr<sensor_msgs::msg::Imu> msg) -> void
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;
}

auto OdomPublisher::vel_subscriber_callback(std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void
{
    if (!msg->velocity.empty()) {
        latest_velocity_ = msg->velocity[0];  // assume front right wheel
    }
}

auto OdomPublisher::publisher_control_loop() -> void
{
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    double dx = latest_velocity_ * wheel_radius * std::cos(yaw_) * dt;
    double dy = latest_velocity_ * wheel_radius * std::sin(yaw_) * dt;

    x_ += dx;
    y_ += dy;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "robot_odom";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = latest_velocity_;
    odom_msg.twist.twist.angular.z = 0.0; // Optional: you could calculate from IMU angular velocity.z
    RCLCPP_INFO(this->get_logger(), "Odom x: %.2f y: %.2f, theta: %.2f", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw_);
    odom_publisher_->publish(odom_msg);
    last_time_ = current_time;
}

OdomPublisher::~OdomPublisher()
{

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
