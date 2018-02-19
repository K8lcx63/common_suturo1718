#include <ros/ros.h>
#include <interactiv_marker_publisher/interactiv_marker_publisher.h>
#include <cmath> 

InteractivMarkerPublisher::InteractivMarkerPublisher(ros::NodeHandle nh):
  nh_(nh),
  server("interactiv_marker_server")
{
  pose_.position.x = 0.5;
  pose_.position.y = 0.5;
  pose_.position.z = 0.5;
  pose_.orientation.w = 1.0;
  tf_timer_ = nh_.createTimer(ros::Duration(0.05),boost::bind(&InteractivMarkerPublisher::updateTf, this, 0, _1));
}

void InteractivMarkerPublisher::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  pose_ = feedback->pose;
}

Marker InteractivMarkerPublisher::makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://grasp/meshes/ja_milch/ja_milch.dae";
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& InteractivMarkerPublisher::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void InteractivMarkerPublisher::updateTf(int, const ros::TimerEvent& event)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose_.position.x, pose_.position.y, pose_.position.z));
  transform.setRotation(tf::Quaternion(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w));
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "ja_milch"));

  tf::StampedTransform transformGripper;
  try
  {
    listener.lookupTransform("/l_gripper_tool_frame", "/base_link", ros::Time(0), transformGripper);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::StampedTransform transformObject;
  try
  {
    listener.lookupTransform("/ja_milch", "/base_link", ros::Time(0), transformObject);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Quaternion gripperQuaternion = transformGripper.getRotation();
  tf::Quaternion objectQuaternion = transformObject.getRotation();
  
  tf::Matrix3x3 m(gripperQuaternion);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  tf::Matrix3x3 m2(objectQuaternion);
  double roll2, pitch2, yaw2;
  m2.getRPY(roll2, pitch2, yaw2);

  double yawDiff = std::abs(yaw-yaw2);
  ROS_INFO_STREAM("yawDiff: " << yawDiff);

  geometry_msgs::PointStamped graspPoint;
  geometry_msgs::PointStamped gripperOrigin;
  gripperOrigin.header.frame_id = "/l_gripper_tool_frame";
  gripperOrigin.header.stamp = ros::Time(0);
  gripperOrigin.point.x = 0.0;
  gripperOrigin.point.y = 0.0;
  gripperOrigin.point.z = 0.0;

  try
  {
    //listener.waitForTransform("/ja_milch", "l_gripper_tool_frame", ros::Time(0), ros::Duration(10.0));
    listener.transformPoint("/ja_milch", gripperOrigin, graspPoint);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_STREAM("x: " << graspPoint.point.x << " y: " << graspPoint.point.y << " z: " << graspPoint.point.z);
}

void InteractivMarkerPublisher::make6DofMarker()
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = 0.5;
  int_marker.pose.position.y = 0.5;
  int_marker.pose.position.z = 0.5;
  int_marker.scale = 0.2;

  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  InteractiveMarkerControl control;

  
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, boost::bind(&InteractivMarkerPublisher::processFeedback, this, _1));
  server.applyChanges();
}