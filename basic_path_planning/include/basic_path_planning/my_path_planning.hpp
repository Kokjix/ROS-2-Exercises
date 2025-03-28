#ifndef MY_PATH_PLANNING
#define MY_PATH_PLANNING

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MyPathPlanning : public rclcpp::Node
{
    private:
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr planner_publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr planner_subscription_;
        geometry_msgs::msg::Twist goal_vel;
        turtlesim::msg::Pose goal;
        turtlesim::msg::Pose obstacle;
        turtlesim::msg::Pose::SharedPtr current_pose;
        float safe_distance;
        float obstacle_distance;
        float goal_distance;
        float angle_to_goal;
        float angle_to_goal_error;
    public:
        MyPathPlanning();
        ~MyPathPlanning();
        void publisher_control_loop();
        float calculate_angle();
        void subscriber_callback(std::shared_ptr<turtlesim::msg::Pose> msg);

};

#endif