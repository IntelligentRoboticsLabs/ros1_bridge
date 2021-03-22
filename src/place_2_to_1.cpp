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
#include "moveit_msgs/PlaceLocation.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/place_location.hpp"


ros::Publisher pub;

void placeCallback(const moveit_msgs::msg::PlaceLocation::SharedPtr ros2_msg)
{
  if (pub.getNumSubscribers() == 0)
    return;

  moveit_msgs::PlaceLocation ros1_msg;
  ros1_msg.id = ros2_msg->id;
  ros1_msg.place_pose.header.frame_id = ros2_msg->place_pose.header.frame_id;
  ros1_msg.place_pose.header.stamp.sec = ros2_msg->place_pose.header.stamp.sec;
  ros1_msg.place_pose.header.stamp.nsec = ros2_msg->place_pose.header.stamp.nanosec;
  ros1_msg.place_pose.pose.position.x = ros2_msg->place_pose.pose.position.x;
  ros1_msg.place_pose.pose.position.y = ros2_msg->place_pose.pose.position.y;
  ros1_msg.place_pose.pose.position.z = ros2_msg->place_pose.pose.position.z;
  ros1_msg.place_pose.pose.orientation.x = ros2_msg->place_pose.pose.orientation.x;
  ros1_msg.place_pose.pose.orientation.y = ros2_msg->place_pose.pose.orientation.y;
  ros1_msg.place_pose.pose.orientation.z = ros2_msg->place_pose.pose.orientation.z;
  ros1_msg.place_pose.pose.orientation.w = ros2_msg->place_pose.pose.orientation.w;
  
  pub.publish(ros1_msg);
}

int main(int argc, char * argv[])
{
  // ROS 2 node and subscriber
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("place_2_to_1");
  auto sub = node->create_subscription<moveit_msgs::msg::PlaceLocation>(
    "input", 100, placeCallback);


  // ROS 1 node and publisher
  ros::init(argc, argv, "place_2_to_1");
  ros::NodeHandle n;
  pub = n.advertise<moveit_msgs::PlaceLocation>("output", 100);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
