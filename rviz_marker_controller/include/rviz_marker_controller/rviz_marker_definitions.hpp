#ifndef RVIZ_MARKER_DEFINITIONS
#define RVIZ_MARKER_DEFINITIONS

#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

class RvizInteractiveMarker : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    geometry_msgs::msg::Pose initial_pose;
public:
    RvizInteractiveMarker();
    void create_interactive_markers();
    void process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback_msg);
    ~RvizInteractiveMarker();
};

#endif