/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#include <math.h>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
static const double QUATERNION_TOLERANCE = 0.1f;
class BasicControls : public rclcpp::Node
{
public:
  BasicControls()
      : Node("basic_controls")
  {
    server = std::make_shared<interactive_markers::InteractiveMarkerServer>("basic_controls",
                                                                            get_node_base_interface(),
                                                                            get_node_clock_interface(),
                                                                            get_node_logging_interface(),
                                                                            get_node_topics_interface(),
                                                                            get_node_services_interface());
  }
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  interactive_markers::MenuHandler menu_handler;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // static const double QUATERNION_TOLERANCE = 0.1f;

  inline void
  applyChanges()
  {
    server->applyChanges();
  }

  visualization_msgs::msg::Marker makeBox(visualization_msgs::msg::InteractiveMarker &msg)
  {
    visualization_msgs::msg::Marker marker;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
  }

  visualization_msgs::msg::InteractiveMarkerControl &makeBoxControl(visualization_msgs::msg::InteractiveMarker &msg)
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
  }

  void frameCallback()
  {
    static uint32_t counter = 0;
    // static tf2_ros::TransformBroadcaster br(*this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2::Transform t;

    rclcpp::Time time = rclcpp::Clock().now();

    t.setOrigin(tf2::Vector3(0.0, 0.0, sin(float(counter) / 140.0) * 2.0));
    t.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    // br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = time;
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "moving_frame";

    tf_broadcaster_->sendTransform(transformStamped);
    t.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    // t.setRotation(tf2::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, float(counter) / 140.0, 0.0);
    t.setRotation(q);
    // br.sendTransform(tf2::StampedTransform(t, time, "base_link", "rotating_frame"));
    geometry_msgs::msg::TransformStamped transformStamped2;
    transformStamped2.header.stamp = time;
    transformStamped2.header.frame_id = "base_link";
    transformStamped2.child_frame_id = "rotating_frame";

    tf_broadcaster_->sendTransform(transformStamped2);

    counter++;
  }

  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if (feedback->mouse_point_valid)
    {
      mouse_point_ss << " at " << feedback->mouse_point.x
                     << ", " << feedback->mouse_point.y
                     << ", " << feedback->mouse_point.z
                     << " in frame " << feedback->header.frame_id;
    }

    switch (feedback->event_type)
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      RCLCPP_INFO_STREAM(get_logger(), s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      RCLCPP_INFO_STREAM(get_logger(), s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      RCLCPP_INFO_STREAM(get_logger(), s.str() << ": pose changed"
                                               << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w << ", " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", " << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: " << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nanosec << " nsec");
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
      RCLCPP_INFO_STREAM(get_logger(), s.str() << ": mouse down" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
      RCLCPP_INFO_STREAM(get_logger(), s.str() << ": mouse up" << mouse_point_ss.str() << ".");
      break;
    }

    server->applyChanges();
  }

  void alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    geometry_msgs::msg::Pose pose = feedback->pose;

    pose.position.x = round(pose.position.x - 0.5) + 0.5;
    pose.position.y = round(pose.position.y - 0.5) + 0.5;

    RCLCPP_INFO_STREAM(get_logger(), feedback->marker_name << ":"
                                                           << " aligning position = "
                                                           << feedback->pose.position.x
                                                           << ", " << feedback->pose.position.y
                                                           << ", " << feedback->pose.position.z
                                                           << " to "
                                                           << pose.position.x
                                                           << ", " << pose.position.y
                                                           << ", " << pose.position.z);

    server->setPose(feedback->marker_name, pose);
    server->applyChanges();
  }

  double rand(double min, double max)
  {
    double t = (double)std::rand() / (double)RAND_MAX;
    return min + t * (max - min);
  }

  void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3 &position, bool show_6dof)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    // pointTFToMsg(const Point& bt_v, geometry_msgs::Point& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();}
    int_marker.pose.position.x = position.x();
    int_marker.pose.position.y = position.y();
    int_marker.pose.position.z = position.z();

    int_marker.scale = 1;

    int_marker.name = "simple_6dof";
    int_marker.description = "Simple 6-DOF Control";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    visualization_msgs::msg::InteractiveMarkerControl control;

    if (fixed)
    {
      int_marker.name += "_fixed";
      int_marker.description += "\n(fixed orientation)";
      control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    {
      std::string mode_text;
      if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D)
        mode_text = "MOVE_3D";
      if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D)
        mode_text = "ROTATE_3D";
      if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D)
        mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if (show_6dof)
    {
      tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
      orien.normalize();
      // tf2::quaternionTFToMsg(orien, control.orientation);
      // maybe not correct
      control.orientation = tf2::toMsg(orien);

      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
      orien.normalize();
      quaternionTFToMsg(orien, control.orientation);

      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
      orien.normalize();
      quaternionTFToMsg(orien, control.orientation);

      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
    if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
      menu_handler.apply(*server, int_marker.name);
  }

  void makeRandomDofMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "6dof_random_axes";
    int_marker.description = "6-DOF\n(Arbitrary Axes)";

    makeBoxControl(int_marker);

    visualization_msgs::msg::InteractiveMarkerControl control;

    for (int i = 0; i < 3; i++)
    {
      tf2::Quaternion orien(rand(-1, 1), rand(-1, 1), rand(-1, 1), rand(-1, 1));
      orien.normalize();
      quaternionTFToMsg(orien, control.orientation);
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
  }
  void makeChessPieceMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "chess_piece";
    int_marker.description = "Chess Piece\n(2D Move + Alignment)";

    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    // make a box which also moves in the plane
    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;
    int_marker.controls.push_back(control);

    // we want to use our special callback function
    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
    // set different callback for POSE_UPDATE feedback
    server->setCallback(int_marker.name, std::bind(&BasicControls::alignMarker, this, _1), visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
  }

  void makePanTiltMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "pan_tilt";
    int_marker.description = "Pan / Tilt";

    makeBoxControl(int_marker);

    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
  }

  void makeMenuMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "context_menu";
    int_marker.description = "Context Menu\n(Right Click)";

    visualization_msgs::msg::InteractiveMarkerControl control;

    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    control.name = "menu_only_control";

    visualization_msgs::msg::Marker marker = makeBox(int_marker);
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
    menu_handler.apply(*server, int_marker.name);
  }

  void makeButtonMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "button";
    int_marker.description = "Button\n(Left Click)";

    visualization_msgs::msg::InteractiveMarkerControl control;

    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    control.name = "button_control";

    visualization_msgs::msg::Marker marker = makeBox(int_marker);
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
  }

  void makeMovingMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "moving";
    int_marker.description = "Marker Attached to a\nMoving Frame";

    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    control.always_visible = true;
    control.markers.push_back(makeBox(int_marker));
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
  }

  static inline void pointTFToMsg(const tf2::Vector3 &bt_v, geometry_msgs::msg::Point &msg_v)
  {
    msg_v.x = bt_v.x();
    msg_v.y = bt_v.y();
    msg_v.z = bt_v.z();
  };
  static inline void quaternionTFToMsg(const tf2::Quaternion &bt, geometry_msgs::msg::Quaternion &msg)
  {
    if (fabs(bt.length2() - 1) > QUATERNION_TOLERANCE)
    {
      tf2::Quaternion bt_temp = bt;
      bt_temp.normalize();
      msg.x = bt_temp.x();
      msg.y = bt_temp.y();
      msg.z = bt_temp.z();
      msg.w = bt_temp.w();
    }
    else
    {
      msg.x = bt.x();
      msg.y = bt.y();
      msg.z = bt.z();
      msg.w = bt.w();
    }
  };

  void makeViewFacingMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "view_facing";
    int_marker.description = "View Facing 6-DOF";

    visualization_msgs::msg::InteractiveMarkerControl control;

    // make a control that rotates around the view axis
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation.w = 1;
    control.name = "rotate";

    int_marker.controls.push_back(control);

    // create a box in the center which should not be view facing,
    // but move in the camera plane.
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    control.independent_marker_orientation = true;
    control.name = "move";

    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;

    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
  }

  void makeQuadrocopterMarker(const tf2::Vector3 &position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "quadrocopter";
    int_marker.description = "Quadrocopter";

    makeBoxControl(int_marker);

    visualization_msgs::msg::InteractiveMarkerControl control;

    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, std::bind(&BasicControls::processFeedback, this, _1));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto basic_controls = std::make_shared<BasicControls>();

  tf2::Vector3 position;
  ///
  position = tf2::Vector3(-3, 3, 0);
  basic_controls->make6DofMarker(false, visualization_msgs::msg::InteractiveMarkerControl::NONE, position, true);
  position = tf2::Vector3(0, 3, 0);
  basic_controls->make6DofMarker(true, visualization_msgs::msg::InteractiveMarkerControl::NONE, position, true);
  position = tf2::Vector3(3, 3, 0);
  basic_controls->makeRandomDofMarker(position);
  position = tf2::Vector3(-3, 0, 0);
  basic_controls->make6DofMarker(false, visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D, position, false);
  position = tf2::Vector3(0, 0, 0);
  basic_controls->make6DofMarker(false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true);
  position = tf2::Vector3(3, 0, 0);
  basic_controls->make6DofMarker(false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D, position, false);
  position = tf2::Vector3(-3, -3, 0);
  basic_controls->makeViewFacingMarker(position);
  position = tf2::Vector3(0, -3, 0);
  basic_controls->makeQuadrocopterMarker(position);
  position = tf2::Vector3(3, -3, 0);
  basic_controls->makeChessPieceMarker(position);
  position = tf2::Vector3(-3, -6, 0);
  basic_controls->makePanTiltMarker(position);
  position = tf2::Vector3(0, -6, 0);
  basic_controls->makeMovingMarker(position);
  position = tf2::Vector3(3, -6, 0);
  basic_controls->makeMenuMarker(position);
  position = tf2::Vector3(0, -9, 0);
  basic_controls->makeButtonMarker(position);
  ///
  basic_controls->applyChanges();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(basic_controls);
  RCLCPP_INFO(basic_controls->get_logger(), "Ready");
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
