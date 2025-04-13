#include "my_vector_field/static_vector_field.hpp"

/**
 * @author: Baran Berk Bagci
 * @date: 10.04.2025
 */

StaticVectorField::StaticVectorField() : Node("static_vector_field_publisher"), count_(0)
{
    step_size = 1e-5;
    k = 1.0;
    virtual_expansion = 5.0; // 0.0003
    iteration_number = 1000;
    goal_point << 80.0, 20.54;
    initial_point << 0.0, 0.0;
    obstacles.resize(5,2);
    obstacles << 6.49, 0.138,
                9.60, 4.334,
                8.01, 6.73,
                3.97, 6.67,
                -1.843, 0.95;
    break_the_loop = false;
    

    position = initial_point;
    position_for_path = initial_point;
    
    RCLCPP_INFO(this->get_logger(), "Goal Point: x=%.2f, y=%.2f", goal_point[0], goal_point[1]);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odom", 
        10, 
        std::bind(&StaticVectorField::subscriber_callback, this, std::placeholders::_1));
    
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/robot/robotnik_base_controller/cmd_vel", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/vector_field_path", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/robot/obstacle_markers", 10);
    marker_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StaticVectorField::publish_obstacle_marker, this));
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&StaticVectorField::publisher_control_loop, this));
}

auto StaticVectorField::publisher_control_loop() -> void
{
    nav_msgs::msg::Path path_msg;

    // for (size_t i = 0; i < iteration_number; i++)
    // {
    velocity_vector = repulsive_force() - attractive_force();
    velocity_norm = velocity_vector.norm();

    if (velocity_norm < 1e-6)
    {
        velocity_norm = 1e-6;
    }
    position += step_size * (velocity_vector / velocity_norm);
    position_for_path += step_size * (velocity_vector / velocity_norm);
    path.push_back(position_for_path);
    // }
    
    path_msg.header.frame_id = "robot_odom";
    path_msg.header.stamp = this->now();

    for (const auto& point : path) 
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "robot_odom";
        pose.header.stamp = this->now();  // Optional, just for completeness
        pose.pose.position.x = point[0];
        pose.pose.position.y = point[1];
        pose.pose.position.z = 0.0;
        path_msg.poses.push_back(pose);
        // RCLCPP_INFO(this->get_logger(), "x:%f, y:%f", point[0], point[1]);

    }
    path_publisher_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Position: x=%f, y=%f", position[0], position[1]);

    angle_to_target = std::atan2(position[1], position[0]);

    auto normalize_angle = [](float angle) -> float 
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };

    float angle_to_goal_error = normalize_angle(angle_to_target - theta);

    linear_speed = std::min(2.0, velocity_norm);
    angular_speed = 0.1 * angle_to_goal_error;

    vel_msg.header.stamp = this->now();
    vel_msg.header.frame_id = "robot_base_controller";

    if (std::abs(angle_to_goal_error) < 0.1)
    {
        vel_msg.twist.linear.x = 2 * linear_speed;
        vel_msg.twist.angular.z = 0.0;
    }
    
    else
    {
        vel_msg.twist.linear.x = 0.0;
        vel_msg.twist.angular.z = angular_speed;
    }
    

    RCLCPP_INFO(this->get_logger(), "Vel: x:%.2f, w:%.4f, angle_er: %.4f, pos_er: %.4f", vel_msg.twist.linear.x, vel_msg.twist.angular.z, angle_to_goal_error,(position - goal_point).norm());

    if ((position - goal_point).norm() < 0.1)
    {
        vel_msg.twist.linear.x = 0.0;
        vel_msg.twist.angular.z = 0.0;
        break_the_loop = true;
    }
    vel_publisher_->publish(vel_msg);
}

auto StaticVectorField::publish_obstacle_marker() -> void
{
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < obstacles.rows(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "robot_odom";
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
    
        marker.pose.position.x = obstacles(i, 0);
        marker.pose.position.y = obstacles(i, 1);
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
    
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
    
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    
        marker_array.markers.push_back(marker);
    }
    marker_publisher_->publish(marker_array);
}

auto StaticVectorField::attractive_force() -> Eigen::Vector2d
{
    Eigen::Vector2d f;
    if ((position - goal_point).norm() < 1e-6)
    {
        f << 0,0;
        return f;
    }
    f = k*(position - goal_point) / (position - goal_point).norm();
    return f;
}

auto StaticVectorField::repulsive_potential(float distance) -> float
{
    float u = 0.0;
    u += std::exp((k*(virtual_expansion - distance)));
    return u;
}

auto StaticVectorField::repulsive_force() -> Eigen::Vector2d
{
    Eigen::Vector2d f_total(0.0, 0.0);
    for (size_t i = 0; i < obstacles.rows(); i++)
    {
        Eigen::Vector2d selected_obstacle = obstacles.row(i);
        float distance = (position - selected_obstacle).norm();

        if (distance < virtual_expansion)
        {
            float U_rep = repulsive_potential(distance);
            float norm_of_a_vector = distance;
            if (norm_of_a_vector < 1e-6)
            {
                distance = 1e-6;
            }
            Eigen::Vector2d f = U_rep * (position - selected_obstacle)/ norm_of_a_vector;
            f_total += f;
        }
    }
    return f_total;
}

auto StaticVectorField::subscriber_callback(std::shared_ptr<nav_msgs::msg::Odometry> msg) -> void
{
    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;

    // RCLCPP_INFO(this->get_logger(),
    //     "Odom Position: [x=%.2f, y=%.2f, z=%.2f], Orientation: [x=%.2f, y=%.2f, z=%.2f, w=%.2f]",
    //     pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w);
    real_pose = msg->pose.pose;
    quarternions = real_pose.orientation;
    theta = get_yaw_from_orientation(quarternions);
}

auto StaticVectorField::get_yaw_from_orientation(const geometry_msgs::msg::Quaternion &q) -> float
{
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

StaticVectorField::~StaticVectorField()
{

}