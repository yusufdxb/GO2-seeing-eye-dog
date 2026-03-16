// ============================================================
// go2_gait_controller/main.cpp
// ============================================================
// Entry point for the GO2 Gait Controller lifecycle node.
// Author: Yusuf Guenena
// ============================================================

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "go2_gait_controller/gait_controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<go2_gait_controller::GaitControllerNode>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
