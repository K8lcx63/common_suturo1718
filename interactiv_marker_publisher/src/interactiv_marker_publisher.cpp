#include <ros/ros.h>
#include <interactiv_marker_publisher/interactiv_marker_publisher.h>

InteractivMarkerPublisher::InteractivMarkerPublisher():
  server("interactiv_marker_server")
{

}

void InteractivMarkerPublisher::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

void InteractivMarkerPublisher::addInteractivMarker(const std::string name, const std::string meshPath)
{
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.name = name;

  visualization_msgs::Marker mesh_marker;
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh_marker.mesh_resource = meshPath;

  visualization_msgs::InteractiveMarkerControl mesh_control;
  mesh_control.always_visible = true;
  mesh_control.markers.push_back(mesh_marker);
  marker.controls.push_back(mesh_control);

  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(rotate_control);

  server.insert(marker, boost::bind(&InteractivMarkerPublisher::processFeedback, this, _1));
  server.applyChanges();
}

void InteractivMarkerPublisher::addInteractivMarker(const std::string name, const float scaleX, const float scaleY, const Color color)
{
  

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = name;
  int_marker.description = "";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = scaleX;
  box_marker.scale.y = scaleY;
  box_marker.scale.z = 0.05;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_3D;

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, boost::bind(&InteractivMarkerPublisher::processFeedback, this, _1));

  // 'commit' changes and send to all clients
  server.applyChanges();
}