#include "my_vector_field/static_vector_field.hpp"

/**
 * @author: Baran Berk Bagci
 * @date: 10.04.2025
 */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticVectorField>();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->break_the_loop)
        {
            break;
        }
    }
    // rclcpp::spin();
    rclcpp::shutdown();
    return 0;
}
