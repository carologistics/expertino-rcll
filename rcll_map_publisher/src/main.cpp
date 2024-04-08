#Copyright(C) 2024 Team Carologistics
#
#Licensed under GPLv2 + license, cf.LICENSE file in project root directory.

#include "rcll_map_publisher/rcll_map_publisher.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto map_publisher = std::make_shared<MapPublisher>();
  rclcpp::spin(map_publisher);
  rclcpp::shutdown();
  return 0;
}
