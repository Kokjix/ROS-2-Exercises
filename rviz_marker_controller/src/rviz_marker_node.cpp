#include "rviz_marker_controller/rviz_marker_definitions.hpp"

RvizInteractiveMarker::RvizInteractiveMarker() : Node("twist_rviz_interactive_marker")
{
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/robotnik_base_controller/cmd_vel_unstamped", 10);
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("twist_marker",
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_topics_interface(),
        this->get_node_services_interface(),
        rclcpp::SystemDefaultsQoS(),
        rclcpp::SystemDefaultsQoS()
    );

    create_interactive_markers();
    server_->applyChanges();
}

auto RvizInteractiveMarker::create_interactive_markers() -> void
{
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "robot_base_footprint";
    int_marker.name = "twist_marker";
    int_marker.description = "Move to publish Twist";
    int_marker.scale = 1.0;

    int_marker.pose.position.x = 0;
    int_marker.pose.position.y = 0;
    int_marker.pose.position.z = 0;

    initial_pose = int_marker.pose;


    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    int_marker.controls.push_back(control);

    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    int_marker.controls.push_back(control);

    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    int_marker.controls.push_back(control);

    server_->insert(int_marker);
    server_->setCallback(int_marker.name, std::bind(&RvizInteractiveMarker::process_feedback, this, std::placeholders::_1));

}

auto RvizInteractiveMarker::process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback_msg) -> void
{
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = feedback_msg->pose.position.x;
    vel_msg.linear.y = feedback_msg->pose.position.y;
    vel_msg.angular.z = tf2::getYaw(feedback_msg->pose.orientation);
    twist_pub_->publish(vel_msg);
    RCLCPP_INFO(this->get_logger(), "Received feedback: x=%.2f, y=%.2f, z=%.2f",
            feedback_msg->pose.position.x,
            feedback_msg->pose.position.y,
            feedback_msg->pose.position.z
                );
    RCLCPP_INFO(this->get_logger(), "Pub Vel: x:%.4f y:%.4f ang_z: %.4f", 
                                    vel_msg.linear.x,
                                    vel_msg.linear.y,
                                    vel_msg.angular.z
                );
    server_->setPose(feedback_msg->marker_name, initial_pose);
    server_->applyChanges();
}

RvizInteractiveMarker::~RvizInteractiveMarker()
{

}