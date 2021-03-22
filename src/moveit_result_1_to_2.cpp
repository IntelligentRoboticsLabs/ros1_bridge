// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"


rclcpp::Publisher<moveit_msgs::msg::MoveItErrorCodes>::SharedPtr pub;


void moveitResultCallback(boost::shared_ptr<moveit_msgs::MoveItErrorCodes> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<moveit_msgs::msg::MoveItErrorCodes>();

  ros2_msg->val = ros1_msg->val;

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_result_1_to_2");
  pub = node->create_publisher<moveit_msgs::msg::MoveItErrorCodes>("output", rclcpp::QoS(100).keep_all().transient_local());

  // ROS 1 node and subscriber
  ros::init(argc, argv, "moveit_result_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("input", 100, moveitResultCallback);

  ros::spin();

  rclcpp::shutdown();

  return 0;
}
