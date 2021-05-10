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

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


ros::Publisher pub;

void twistCallback(const geometry_msgs::msg::Twist::SharedPtr ros2_msg)
{
  if (pub.getNumSubscribers() == 0)
    return;

  geometry_msgs::Twist ros1_msg;
  ros1_msg.linear.x = ros2_msg->linear.x;
  ros1_msg.linear.y = ros2_msg->linear.y;
  ros1_msg.linear.z = ros2_msg->linear.z;
  ros1_msg.angular.x = ros2_msg->angular.x;
  ros1_msg.angular.y = ros2_msg->angular.y;
  ros1_msg.angular.z = ros2_msg->angular.z;
  pub.publish(ros1_msg);
}

int main(int argc, char * argv[])
{
  // ROS 2 node and subscriber
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("twist_2_to_1");
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "input", 100, twistCallback);


  // ROS 1 node and publisher
  ros::init(argc, argv, "twist_2_to_1");
  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::Twist>("output", 100);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
