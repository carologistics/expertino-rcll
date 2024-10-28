// Copyright(C) 2024 Team Carologistics
//
// Licensed under GPLv2 + license, cf.LICENSE file in project root directory.

#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <google/protobuf/message.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/peer.h>
#include <tf2_ros/transform_broadcaster.h>

class MapPublisher : public rclcpp::Node {
public:
  MapPublisher();

private:
  struct machine_info {
    float x;
    float y;
    int rotation;
  };
  void handle_peer_msg(boost::asio::ip::udp::endpoint &endpoint,
                       uint16_t component_id, uint16_t msg_type,
                       std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint,
                              std::string msg);

  void publish_map();
  void publish_tf();
  void updateOccupancyGrid(const machine_info &machine_info,
                           int occupancy_value);
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  float get_x(std::string &zone_name);
  float get_y(std::string &zone_name);
  float resolution_;
  float machine_length_ = 0.8;
  float machine_width_ = 0.3;
  float offset_ = 7.;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> private_peer_;
  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> public_peer_;
  std::shared_ptr<protobuf_comm::MessageRegister> message_register_;
  std::unordered_map<std::string, machine_info> machine_map_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  void rotate_point(double x, double y, double cx, double cy, double theta,
                    double &newX, double &newY);
};
