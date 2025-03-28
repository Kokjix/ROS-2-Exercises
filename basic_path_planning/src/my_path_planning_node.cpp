#include "basic_path_planning/my_path_planning.hpp"

MyPathPlanning::MyPathPlanning() : Node("my_path_publisher"), count_(0)
{
    obstacle.x = 3.0;
    obstacle.y = 2.0;

    goal.x = 1.0;
    goal.y = 1.0;

    safe_distance = 0.5;

    RCLCPP_INFO(this->get_logger(), "Obstacle Pose: x=%.2f, y=%.2f, theta=%.2f", 
            obstacle.x, obstacle.y, obstacle.theta);

    RCLCPP_INFO(this->get_logger(), "Goal Pose: x=%.2f, y=%.2f, theta=%.2f", 
    goal.x, goal.y, goal.theta);
    

    planner_subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&MyPathPlanning::subscriber_callback, this, std::placeholders::_1));
    planner_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MyPathPlanning::publisher_control_loop, this));

}

auto MyPathPlanning::subscriber_callback(std::shared_ptr<turtlesim::msg::Pose> msg) -> void
{
    current_pose = msg;
}

auto MyPathPlanning::publisher_control_loop() -> void
{
    if (!current_pose) 
    {
        return;
    }

    obstacle_distance = sqrt(pow((obstacle.x - current_pose->x),2) + pow((obstacle.y - current_pose->y),2));
    goal_distance = sqrt(pow((goal.x - current_pose->x),2) + pow((goal.y - current_pose->y),2));

    if (goal_distance < 0.5)
    {
        goal_vel.linear.x = 0.0;
        goal_vel.angular.z = 0.0;
    }

    else if (obstacle_distance < safe_distance)
    {
        goal_vel.angular.z = 3.14;
        rclcpp::sleep_for(std::chrono::seconds(1));
        goal_vel.linear.x = 1.0;
        // rclcpp::sleep_for(std::chrono::seconds(1));
    }
    
    else
    {
        angle_to_goal = calculate_angle();
        angle_to_goal_error = angle_to_goal - current_pose->theta;
        goal_vel.linear.x = 1.0;
        goal_vel.angular.z = 2 * angle_to_goal_error;
    }

    RCLCPP_INFO(this->get_logger(), "GOAL Vel: linear_x=%.2f, angular_z=%.2f", 
            goal_vel.linear.x, goal_vel.angular.z);

    this->planner_publisher_->publish(goal_vel);
    

}


auto MyPathPlanning::calculate_angle() -> float
{
    return atan2(goal.y - current_pose->y, goal.x - current_pose->x);
}

MyPathPlanning::~MyPathPlanning()
{

}