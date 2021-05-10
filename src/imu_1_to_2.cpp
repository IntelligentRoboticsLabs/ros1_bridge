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
#include "sensor_msgs/Imu.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"


rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;


void imuCallback(boost::shared_ptr<sensor_msgs::Imu> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<sensor_msgs::msg::Imu>();

  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header.stamp = rclcpp::Time(ros1_msg->header.stamp.toNSec());
  
  ros2_msg->orientation.x = ros1_msg->orientation.x;
  ros2_msg->orientation.y = ros1_msg->orientation.y;
  ros2_msg->orientation.z = ros1_msg->orientation.z;
  ros2_msg->orientation.w = ros1_msg->orientation.w;
  std::array<double, 9> o_data, a_data, l_data;
  int i = 0, j = 0, k = 0;
  
  for (auto & data : ros1_msg->orientation_covariance)
  {
    o_data[i] = data;
    i++;
  }
  ros2_msg->orientation_covariance = o_data;

  for (auto & data : ros1_msg->angular_velocity_covariance)
  {
    a_data[j] = data;
    j++;
  }
  ros2_msg->angular_velocity_covariance = a_data;

  for (auto & data : ros1_msg->linear_acceleration_covariance)
  {
    l_data[k] = data;
    k++;
  }
  ros2_msg->linear_acceleration_covariance = l_data;

  ros2_msg->angular_velocity.x = ros1_msg->angular_velocity.x;
  ros2_msg->angular_velocity.y = ros1_msg->angular_velocity.y;
  ros2_msg->angular_velocity.z = ros1_msg->angular_velocity.z;
  ros2_msg->linear_acceleration.x = ros1_msg->linear_acceleration.x;
  ros2_msg->linear_acceleration.y = ros1_msg->linear_acceleration.y;
  ros2_msg->linear_acceleration.z = ros1_msg->linear_acceleration.z;

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("imu_1_to_2");
  pub = node->create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::SensorDataQoS());

  // ROS 1 node and subscriber
  ros::init(argc, argv, "imu_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("input", 100, imuCallback);

  ros::spin();

  rclcpp::shutdown();

  return 0;
}
