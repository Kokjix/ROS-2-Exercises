#ifndef STATIC_VECTOR_FIELD
#define STATIC_VECTOR_FIELD

#include <iostream>
#include <memory>
#include <chrono>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

/**
 * @author: Baran Berk Bagci
 * @date: 10.04.2025
 */

class StaticVectorField : public rclcpp::Node
{
    private:
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr marker_timer_;
        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        geometry_msgs::msg::TwistStamped vel_msg;
        geometry_msgs::msg::Pose real_pose;
        geometry_msgs::msg::Quaternion quarternions;
        nav_msgs::msg::Path path_msg;
        float k; // Coefficient of repulsive and attractive force
        float virtual_expansion; // Coefficient of attractive force
        float step_size;
        float angle_to_target;
        float linear_speed;
        float angular_speed;
        float theta;
        double velocity_norm;
        double velocity_norm_for_path;
        Eigen::Vector2d velocity_vector;
        Eigen::Vector2d velocity_vector_for_path;
        size_t iteration_number;
        Eigen::Vector2d goal_point;
        Eigen::Vector2d initial_point;
        Eigen::MatrixXd obstacles;
        Eigen::Vector2d position;
        Eigen::Vector2d position_for_path;
        std::vector<Eigen::Vector2d> path;
    public:
        bool break_the_loop;
        StaticVectorField();
        void calculate_points();
        void publisher_control_loop();
        void subscriber_callback(std::shared_ptr<nav_msgs::msg::Odometry> msg);
        void publish_obstacle_marker();
        Eigen::Vector2d attractive_force();
        float repulsive_potential(float distance);
        Eigen::Vector2d repulsive_force();
        float get_yaw_from_orientation(const geometry_msgs::msg::Quaternion &q);
        ~StaticVectorField();
};

#endif