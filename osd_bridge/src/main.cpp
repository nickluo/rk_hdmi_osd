#include <rclcpp/rclcpp.hpp>

#include "osd_bridge/bridge_node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<osd_bridge::OsdBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    /* node destruction stops the render pipeline */
}
