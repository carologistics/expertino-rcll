// Copyright(C) 2024 Team Carologistics
//
// Licensed under GPLv2 + license, cf.LICENSE file in project root directory.

#include "rcll_map_publisher/rcll_map_publisher.hpp"
#include "rcll_protobuf_cpp/MachineInfo.pb.h"

#include <boost/bind/bind.hpp>
#include <boost/signals2.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

MapPublisher::MapPublisher() : Node("map_publisher") {
  declare_parameter<std::string>("peer_address", "127.0.0.1");
  declare_parameter<unsigned short>("recv_port_private", 4441);
  declare_parameter<unsigned short>("recv_port_public", 4444);
  declare_parameter<std::string>("crypto_key", "randomkey");
  declare_parameter<float>("resolution", 0.05);
  declare_parameter<std::string>(
      "proto_path",
      ament_index_cpp::get_package_share_directory("rcll_protobuf_msgs") +
          "/rcll-protobuf-msgs/");

  std::string peer_address = this->get_parameter("peer_address").as_string();
  unsigned short recv_port_private =
      this->get_parameter("recv_port_private").as_int();
  unsigned short recv_port_public =
      this->get_parameter("recv_port_public").as_int();
  get_parameter<float>("resolution", resolution_);

  RCLCPP_INFO(
      this->get_logger(),
      "Listening to %s on %i (public) and %i (private), using proto Path %s",
      peer_address.c_str(), recv_port_public, recv_port_private,
      this->get_parameter("proto_path").as_string().c_str());

  // setup protobuf communication
  std::vector<std::string> proto_path = {
      this->get_parameter("proto_path").as_string()};
  message_register_ =
      std::make_shared<protobuf_comm::MessageRegister>(proto_path);
  private_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(
      peer_address, recv_port_private, message_register_.get(),
      this->get_parameter("crypto_key").as_string().c_str());

  private_peer_->signal_received().connect(
      boost::bind(&MapPublisher::handle_peer_msg, this, boost::placeholders::_1,
                  boost::placeholders::_2, boost::placeholders::_3,
                  boost::placeholders::_4));
  private_peer_->signal_recv_error().connect(
      boost::bind(&MapPublisher::handle_peer_recv_error, this,
                  boost::placeholders::_1, boost::placeholders::_2));
  public_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(
      peer_address, recv_port_public, message_register_.get());

  public_peer_->signal_received().connect(
      boost::bind(&MapPublisher::handle_peer_msg, this, boost::placeholders::_1,
                  boost::placeholders::_2, boost::placeholders::_3,
                  boost::placeholders::_4));
  public_peer_->signal_recv_error().connect(
      boost::bind(&MapPublisher::handle_peer_recv_error, this,
                  boost::placeholders::_1, boost::placeholders::_2));

  // setup occupancy grid and tf broadcasting
  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(10).durability_volatile().transient_local());

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
    this->publish_map();
    this->publish_tf();
  });
}

void MapPublisher::rotate_point(double x, double y, double cx, double cy,
                                double theta, double &newX, double &newY) {
  // Calculate the new coordinates after rotation
  newX = (x - cx) * cos(theta) - (y - cy) * sin(theta) + cx;
  newY = (x - cx) * sin(theta) + (y - cy) * cos(theta) + cy;
}

void MapPublisher::updateOccupancyGrid(const machine_info &machine_info,
                                       int occupancy_value) {
  // Convert real-world coordinates to grid indices
  int grid_x = static_cast<int>((machine_info.x + offset_) / resolution_);
  int grid_y = static_cast<int>((machine_info.y) / resolution_);
  RCLCPP_INFO(this->get_logger(), "%i,%i and %f,%f with res: %f", grid_x,
              grid_y, machine_info.x, machine_info.y, resolution_);

  // Calculate the grid indices of the corners of the bounding box
  int xmin = grid_x - (machine_width_ / resolution_ * 0.5);
  int ymin = grid_y - (machine_length_ / resolution_ * 0.5);
  int xmax = grid_x + (machine_width_ / resolution_ * 0.5);
  int ymax = grid_y + (machine_length_ / resolution_ * 0.5);

  // Rotate the bounding box corners around the center point
  double angle_rad =
      machine_info.rotation * M_PI / 180.0; // Convert angle to radians
  int cx = grid_x;
  int cy = grid_y;

  // Rotate each corner point of the rectangle around the center of rotation
  double rotated_xmin, rotated_ymin;
  rotate_point(xmin, ymin, cx, cy, angle_rad, rotated_xmin, rotated_ymin);

  double rotated_xmax, rotated_ymax;
  rotate_point(xmax, ymax, cx, cy, angle_rad, rotated_xmax, rotated_ymax);

  double rotated_l_x, rotated_l_y;
  rotate_point(xmin, ymax, cx, cy, angle_rad, rotated_l_x, rotated_l_y);

  double rotated_r_x, rotated_r_y;
  rotate_point(xmax, ymin, cx, cy, angle_rad, rotated_r_x, rotated_r_y);

  // Find the minimum and maximum x and y coordinates among the rotated corner
  // points
  int bbox_xmin = std::min(std::min(rotated_xmin, rotated_xmax),
                           std::min(rotated_r_x, rotated_l_x));
  int bbox_ymin = std::min(std::min(rotated_ymin, rotated_ymax),
                           std::min(rotated_l_y, rotated_r_y));
  int bbox_xmax = std::max(std::max(rotated_xmin, rotated_xmax),
                           std::max(rotated_r_x, rotated_l_x));
  int bbox_ymax = std::max(std::max(rotated_ymin, rotated_ymax),
                           std::max(rotated_l_y, rotated_r_y));

  // Iterate over all integer coordinates within the bounding box
  for (int x = bbox_xmin; x <= bbox_xmax; ++x) {
    for (int y = bbox_ymin; y <= bbox_ymax; ++y) {
      // Rotate the point around the center of the rectangle by the negative of
      // the rotation angle
      double rotated_x, rotated_y;
      rotate_point(x, y, cx, cy, -angle_rad, rotated_x, rotated_y);
      if (rotated_x >= xmin && rotated_x <= xmax && rotated_y >= ymin &&
          rotated_y <= ymax) {

        int index = y * occupancy_grid_.info.width + x;
        // Calculate the index of the cell in the data array

        // Set the occupancy value for the calculated index
        occupancy_grid_.data[index] = occupancy_value;
      }
    }
  }
}

void MapPublisher::handle_peer_msg(
    boost::asio::ip::udp::endpoint &, uint16_t, uint16_t,
    std::shared_ptr<google::protobuf::Message> msg_ptr) {
  if (!msg_ptr) {
    // Invalid message pointer
    return;
  }

  const google::protobuf::Descriptor *desc = msg_ptr->GetDescriptor();
  // Process the message based on its type
  if (desc->name() == "MachineInfo") {
    const llsf_msgs::MachineInfo *machine_info_msg =
        dynamic_cast<const llsf_msgs::MachineInfo *>(msg_ptr.get());
    if (machine_info_msg) {

      bool grid_updated = false;
      for (const auto &machine : machine_info_msg->machines()) {
        if (machine.has_rotation() && machine.has_zone()) {
          // Rotation field is set, retrieve its value
          int rotation =
              machine.rotation(); // Assuming rotation is an integer field
          std::string zone_name = llsf_msgs::Zone_Name(machine.zone());
          // Calculate x, y coordinates
          float x = get_x(zone_name);
          float y = get_y(zone_name);
          // Check if machine already exists in machine_map_
          auto it = machine_map_.find(machine.name());
          if (it != machine_map_.end()) {
            // Machine already exists, check for changes
            if (it->second.x != x || it->second.y != y ||
                it->second.rotation != rotation) {
              // Update machine information in machine_map_
              it->second.x = x;
              it->second.y = y;
              it->second.rotation = rotation;
              RCLCPP_INFO(this->get_logger(),
                          "Machine '%s' updated: %f,%f with rot %i",
                          machine.name().c_str(), x, y, rotation);
              grid_updated = true; // Set flag to indicate grid needs updating
            }
          } else {
            // New machine entry
            machine_map_[machine.name()] = {x, y, rotation};
            RCLCPP_INFO(this->get_logger(),
                        "New machine '%s' added: %f,%f with rot %i",
                        machine.name().c_str(), x, y, rotation);
            grid_updated = true; // Set flag to indicate grid needs updating
          }
        }
      }
      // Update occupancy grid and publish if there are changes
      if (grid_updated) {
        // Initialize the map with zeros (unknown)
        occupancy_grid_.info.resolution = resolution_;
        occupancy_grid_.info.width = 14 / resolution_;
        occupancy_grid_.info.height = 8 / resolution_;
        occupancy_grid_.data.resize(occupancy_grid_.info.width *
                                    occupancy_grid_.info.height);
        std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), 0);
        for (const auto &pair : machine_map_) {
          updateOccupancyGrid(pair.second, 100);
        }
        publish_map();
        publish_tf();
      }
    }

  } else {
    RCLCPP_INFO(this->get_logger(), "Received %s", desc->name().c_str());
    // Handle other message types as needed
  }
}

float MapPublisher::get_x(std::string &zone_name) {
  float x = 0;
  char first_char = zone_name[0];
  float first_digit = zone_name[3] - '0';

  // Determine the sign based on the first character
  float sign = (first_char == 'C') ? 1 : -1;

  // Calculate the x-coordinate of center
  x = sign * (first_digit - 0.5);

  return x;
}
float MapPublisher::get_y(std::string &zone_name) {
  // Calculate y-coordinate of center
  return zone_name[4] - '0' - 0.5;
}

void MapPublisher::handle_peer_recv_error(
    boost::asio::ip::udp::endpoint &endpoint, std::string msg) {
  RCLCPP_INFO(
      this->get_logger(), "Failed to receive peer message from %s:%i:%s",
      endpoint.address().to_string().c_str(), endpoint.port(), msg.c_str());
}
void MapPublisher::publish_tf() {
  // Publish transforms for each machine position
  for (const auto &pair : machine_map_) {
    const auto &machine_info = pair.second;

    // Calculate the rotation angle in radians
    double rotation_radians = machine_info.rotation * M_PI / 180.0;

    // Calculate the translation for the input point
    tf2::Vector3 translation_input(std::cos(rotation_radians) * 0.4,
                                   std::sin(rotation_radians) * 0.4, 0.0);
    translation_input += tf2::Vector3(machine_info.x, machine_info.y, 0.0);

    // Calculate the direction vector from the input point to the center
    tf2::Vector3 direction_input(machine_info.x, machine_info.y, 0.0);

    // Create a transform message for machine input
    geometry_msgs::msg::TransformStamped transform_input_msg;
    transform_input_msg.header.stamp = this->now(); // Use current ROS time
    transform_input_msg.header.frame_id = "map";    // Parent frame ID
    transform_input_msg.child_frame_id =
        pair.first + "-INPUT"; // Child frame ID
    transform_input_msg.transform.translation.x = translation_input.x();
    transform_input_msg.transform.translation.y = translation_input.y();
    transform_input_msg.transform.translation.z = translation_input.z();

    // Set the orientation to point towards the center
    tf2::Quaternion quaternion_input;
    quaternion_input.setRPY(
        0.0, 0.0,
        std::atan2(direction_input.y() - translation_input.y(),
                   direction_input.x() - translation_input.x()));
    transform_input_msg.transform.rotation = tf2::toMsg(quaternion_input);

    // Publish the transform for the input point
    tf_broadcaster_->sendTransform(transform_input_msg);

    // Calculate the translation for the output point
    tf2::Vector3 translation_output(-std::cos(rotation_radians) * 0.4,
                                    -std::sin(rotation_radians) * 0.4, 0.0);
    translation_output += tf2::Vector3(machine_info.x, machine_info.y, 0.0);

    // Calculate the direction vector from the output point to the center
    tf2::Vector3 direction_output(machine_info.x, machine_info.y, 0.0);

    // Create a transform message for machine output
    geometry_msgs::msg::TransformStamped transform_output_msg;
    transform_output_msg.header.stamp = this->now(); // Use current ROS time
    transform_output_msg.header.frame_id = "map";    // Parent frame ID
    transform_output_msg.child_frame_id =
        pair.first + "-OUTPUT"; // Child frame ID
    transform_output_msg.transform.translation.x = translation_output.x();
    transform_output_msg.transform.translation.y = translation_output.y();
    transform_output_msg.transform.translation.z = translation_output.z();

    // Set the orientation to point towards the center
    tf2::Quaternion quaternion_output;
    quaternion_output.setRPY(
        0.0, 0.0,
        std::atan2(direction_output.y() - translation_output.y(),
                   direction_output.x() - translation_output.x()));
    transform_output_msg.transform.rotation = tf2::toMsg(quaternion_output);

    // Publish the transform for the output point
    tf_broadcaster_->sendTransform(transform_output_msg);
  }
}

void MapPublisher::publish_map() {

  // Set the frame ID
  occupancy_grid_.header.frame_id =
      "map"; // Replace "map_frame" with your desired frame ID
             //
  occupancy_grid_.header.stamp = this->get_clock()->now();
  occupancy_grid_.info.origin.position.x = -7.0;
  // Set the timestamp
  occupancy_grid_.header.stamp =
      this->get_clock()->now(); // Use current ROS time

  publisher_->publish(occupancy_grid_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto map_publisher = std::make_shared<MapPublisher>();
  rclcpp::spin(map_publisher);
  rclcpp::shutdown();
  return 0;
}
