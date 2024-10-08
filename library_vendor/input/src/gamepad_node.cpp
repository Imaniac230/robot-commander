#include <rclcpp/rclcpp.hpp>

#include <input/GamepadNode.h>


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GamepadNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
