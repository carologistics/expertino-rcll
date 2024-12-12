/***************************************************************************
 *  NavGraphSetBoundingBox.hpp
 *
 *  Automatically Generated: 2024-01-06 13:07:48
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef CX_FEATURES__NAVGRAPHSETBOUNDINGBOX_HPP_
#define CX_FEATURES__NAVGRAPHSETBOUNDINGBOX_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include "cx_utils/NodeThread.hpp"

#include "expertino_msgs/srv/nav_graph_interface_set_bounding_box.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cx {

class NavGraphSetBoundingBox : public ClipsFeature, public rclcpp::Node {
public:
  NavGraphSetBoundingBox();
  ~NavGraphSetBoundingBox();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
  std::thread spin_thread_;
  std::map<
      std::string,
      std::map<std::string,
               rclcpp::Client<expertino_msgs::srv::
                                  NavGraphInterfaceSetBoundingBox>::SharedPtr>>
      request_clients_;
  std::map<
      std::string,
      std::map<std::string,
               std::shared_ptr<expertino_msgs::srv::
                                   NavGraphInterfaceSetBoundingBox::Request>>>
      requests_;
  void create_request(const std::string &env_name,
                      const std::string &service_name);

  void set_field_request(const std::string &env_name,
                         const std::string &service_name,
                         const std::string &field_name, CLIPS::Value value);

  void set_array_request(const std::string &env_name,
                         const std::string &service_name,
                         const std::string &field_name, CLIPS::Values values);

  void request_from_node(const std::string &env_name,
                         const std::string &service_name);

  void service_callback(
      rclcpp::Client<expertino_msgs::srv::NavGraphInterfaceSetBoundingBox>::
          SharedFuture response,
      std::string service_name, std::string env_name);
};
} // namespace cx
#endif // !CX_FEATURES__NAVGRAPHSETBOUNDINGBOX_HPP_