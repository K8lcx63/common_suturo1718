#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <marker_publisher/marker_publisher.h>

MarkerPublisher::MarkerPublisher(const std::string ns, const Color c):
name_space(ns),
color(c)
{
    ros::NodeHandle nh;
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
}

void MarkerPublisher::publishVisualizationMarker(geometry_msgs::PointStamped point)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = point.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.ns = name_space;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
       
    switch(color)
    {
        case Color::BLACK: marker.color.r = 0.0;
                           marker.color.g = 0.0;
                           marker.color.b = 0.0;
        break;
        case Color::WHITE: marker.color.r = 1.0;
                           marker.color.g = 1.0;
                           marker.color.b = 1.0;
        break;
        case Color::BROWN: marker.color.r = 0.54;
                           marker.color.g = 0.27;
                           marker.color.b = 0.07;
        break;
        case Color::RED: marker.color.r = 1.0;
                         marker.color.g = 0.0;
                         marker.color.b = 0.0;
        break;
        case Color::GREEN: marker.color.r = 0.0;
                           marker.color.g = 0.93;
                           marker.color.b = 0.0;
        break;
        case Color::BLUE: marker.color.r = 0.0;
                          marker.color.g = 0.0;
                          marker.color.b = 1.0;
        break;
        case Color::YELLOW: marker.color.r = 1.0;
                            marker.color.g = 1.0;
                            marker.color.b = 0.0;
        break;
        case Color::ORANGE: marker.color.r = 1.0;
                            marker.color.g = 0.54;
                            marker.color.b = 0.0;
        break;
        case Color::PURPLE: marker.color.r = 0.54;
                            marker.color.g = 0.0;
                            marker.color.b = 0.54;
        break;
    }
    vis_pub.publish(marker);
}