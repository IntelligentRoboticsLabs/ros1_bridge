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
#include "vision_msgs/Detection3DArray.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"


rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub;


void detectionCallback(boost::shared_ptr<vision_msgs::Detection3DArray> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<vision_msgs::msg::Detection3DArray>();

  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header.stamp = rclcpp::Time(ros1_msg->header.stamp.toNSec());
  
  ros2_msg->detections.resize(ros1_msg->detections.size());
  for (size_t i = 0; i < ros1_msg->detections.size(); i++) 
  {
    // ros2_msg->detections[i].id = id field does not exists in ROS1 msg
    ros2_msg->detections[i].header.frame_id = ros1_msg->detections[i].header.frame_id;
    ros2_msg->detections[i].header.stamp = 
      rclcpp::Time(ros1_msg->detections[i].header.stamp.toNSec());
    ros2_msg->detections[i].bbox.center.position.x = 
      ros1_msg->detections[i].bbox.center.position.x;
    ros2_msg->detections[i].bbox.center.position.y = 
      ros1_msg->detections[i].bbox.center.position.y;
    ros2_msg->detections[i].bbox.center.position.z = 
      ros1_msg->detections[i].bbox.center.position.z;
    ros2_msg->detections[i].bbox.center.orientation.x = 
      ros1_msg->detections[i].bbox.center.orientation.x;
    ros2_msg->detections[i].bbox.center.orientation.y = 
      ros1_msg->detections[i].bbox.center.orientation.y;
    ros2_msg->detections[i].bbox.center.orientation.z = 
      ros1_msg->detections[i].bbox.center.orientation.z;
    ros2_msg->detections[i].bbox.center.orientation.w = 
      ros1_msg->detections[i].bbox.center.orientation.w;
    ros2_msg->detections[i].bbox.size.x = ros1_msg->detections[i].bbox.size.x;
    ros2_msg->detections[i].bbox.size.y = ros1_msg->detections[i].bbox.size.y;
    ros2_msg->detections[i].bbox.size.z = ros1_msg->detections[i].bbox.size.z;

    ros2_msg->detections[i].results.resize(ros1_msg->detections[i].results.size());
    for (size_t j = 0; j < ros1_msg->detections[i].results.size(); j++) 
    {
      ros2_msg->detections[i].results[j].hypothesis.score = 
        ros1_msg->detections[i].results[j].score;
      ros2_msg->detections[i].results[j].hypothesis.class_id = 
        std::to_string(ros1_msg->detections[i].results[j].id);
      ros2_msg->detections[i].results[j].pose.pose.position.x =
        ros1_msg->detections[i].results[j].pose.pose.position.x;
      ros2_msg->detections[i].results[j].pose.pose.position.y =
        ros1_msg->detections[i].results[j].pose.pose.position.y;
      ros2_msg->detections[i].results[j].pose.pose.position.z =
        ros1_msg->detections[i].results[j].pose.pose.position.z;
      ros2_msg->detections[i].results[j].pose.pose.orientation.x =
        ros1_msg->detections[i].results[j].pose.pose.orientation.x;
      ros2_msg->detections[i].results[j].pose.pose.orientation.y =
        ros1_msg->detections[i].results[j].pose.pose.orientation.y;
      ros2_msg->detections[i].results[j].pose.pose.orientation.z =
        ros1_msg->detections[i].results[j].pose.pose.orientation.z;
      ros2_msg->detections[i].results[j].pose.pose.orientation.w =
        ros1_msg->detections[i].results[j].pose.pose.orientation.w;
      
      // covariance not added beacause we does not need by now
    }
  }

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dope_1_to_2");
  pub = node->create_publisher<vision_msgs::msg::Detection3DArray>("output", rclcpp::SensorDataQoS());

  // ROS 1 node and subscriber
  ros::init(argc, argv, "dope_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("input", 100, detectionCallback);

  ros::spin();

  rclcpp::shutdown();

  return 0;
}
