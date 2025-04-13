#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

class OdomPublisher : public rclcpp::Node
{
    private:
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        sensor_msgs::msg::Imu::SharedPtr imu_msg;
        nav_msgs::msg::Odometry odom_msg;
        float x,y, vx, vy;
        double theta;
        rclcpp::Time last_time;
    public:
        OdomPublisher();
        void subscriber_callback(std::shared_ptr<sensor_msgs::msg::Imu> msg);
        void publisher_control_loop();
        ~OdomPublisher();
};

OdomPublisher::OdomPublisher() : Node("odom_publisher"), count_(0)
{
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    vx = 0.0;
    vy = 0.0;
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/robot/imu/data", 
        10, std::bind(&OdomPublisher::subscriber_callback, this, std::placeholders::_1));
    
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odom", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&OdomPublisher::publisher_control_loop, this));

}

auto OdomPublisher::subscriber_callback(std::shared_ptr<sensor_msgs::msg::Imu> msg) -> void
{
    imu_msg = msg;
}

auto OdomPublisher::publisher_control_loop() -> void
{
    auto current_time = this->now();
    if (last_time.nanoseconds() == 0)
    {
        last_time = current_time;
        return;
    }
    auto dt = (current_time - last_time).seconds();

    float omega = imu_msg->angular_velocity.z;
    float ax = imu_msg->linear_acceleration.x;
    float ay = imu_msg->linear_acceleration.y;

    theta += omega*dt;

    /* I only need theta yaw angle so I do not calculate x and y and using only IMU data 
        cause drifting problem */

    // float ax_world = ax * cos(theta) - ay * sin(theta);
    // float ay_world = ax * sin(theta) + ay * cos(theta);

    // Integrate velocity
    // vx += ax_world * dt;
    // vy += ay_world * dt;

    // x += vx * cos(theta) * dt;
    // y += vy * sin(theta) * dt;

    x = 0.0;
    y = 0.0;

    last_time = current_time;

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "robot_odom";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = omega;
    RCLCPP_INFO(this->get_logger(), "Odom x: %.2f y: %.2f, theta: %.2f", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, theta);
    odom_publisher_->publish(odom_msg);
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
