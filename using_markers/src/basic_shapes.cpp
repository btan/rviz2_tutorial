/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("basic_shapes");
  rclcpp::Rate r(1);

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::msg::Marker::CUBE;

  while (rclcpp::ok())
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration(0, 0);
    while (marker_pub->get_subscription_count() < 1)
    {
      if (!rclcpp::ok())
      {
        return 0;
      }
      RCLCPP_WARN_ONCE(node->get_logger(), "Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub->publish(marker);

    switch (shape)
    {
    case visualization_msgs::msg::Marker::CUBE:
      shape = visualization_msgs::msg::Marker::SPHERE;
      break;
    case visualization_msgs::msg::Marker::SPHERE:
      shape = visualization_msgs::msg::Marker::ARROW;
      break;
    case visualization_msgs::msg::Marker::ARROW:
      shape = visualization_msgs::msg::Marker::CYLINDER;
      break;
    case visualization_msgs::msg::Marker::CYLINDER:
      shape = visualization_msgs::msg::Marker::CUBE;
      break;
    }
    r.sleep();
  }

  //return 0;
}
