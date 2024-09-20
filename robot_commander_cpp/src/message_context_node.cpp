#include <rclcpp/rclcpp.hpp>

#include <robot_commander_cpp/nodes/MessageContextNode.hpp>


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessageContextNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
