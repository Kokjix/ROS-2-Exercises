#include "rviz_marker_controller/rviz_marker_definitions.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RvizInteractiveMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
