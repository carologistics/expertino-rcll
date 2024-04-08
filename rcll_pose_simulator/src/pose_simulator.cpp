#Copyright(C) 2024 Team Carologistics
#
#Licensed under GPLv2 + license, cf.LICENSE file in project root directory.

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
class TFNode : public rclcpp::Node {
public:
  TFNode() : Node("tf_node") {
    // Read start pose parameters
    double start_x = this->declare_parameter("start_x", 1.0);
    double start_y = this->declare_parameter("start_y", 5.0);
    double start_z = this->declare_parameter("start_z", 0.0);
    double start_roll = this->declare_parameter("start_roll", 0.0);
    double start_pitch = this->declare_parameter("start_pitch", 0.0);
    double start_yaw = this->declare_parameter("start_yaw", 0.0);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10,
        std::bind(&TFNode::pathCallback, this, std::placeholders::_1));

    // Publish start pose
    publishStartPose(start_x, start_y, start_z, start_roll, start_pitch,
                     start_yaw);
  }

private:
  void publishStartPose(double x, double y, double z, double roll, double pitch,
                        double yaw) {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();
    tf_broadcaster_->sendTransform(transformStamped);
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty()) {
      auto pose = msg->poses.back().pose;

      // Extracting yaw angle from the quaternion
      tf2::Quaternion quat;
      tf2::fromMsg(pose.orientation, quat);
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      // Publish start pose
      publishStartPose(pose.position.x, pose.position.y, pose.position.z, roll,
                       pitch, yaw);
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
