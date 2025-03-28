#include "basic_path_planning/my_path_planning.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPathPlanning>());
    rclcpp::shutdown();
    return 0;
}
