#ifndef INTERACTIV_MARKER_PUBLISHER_H
#define INTERACTIV_MARKER_PUBLISHER_H
#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace visualization_msgs;

enum class Color {BLACK, WHITE, BROWN, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE};

class InteractivMarkerPublisher
{
private:
    interactive_markers::InteractiveMarkerServer server;
    tf::TransformBroadcaster br_;
    ros::NodeHandle nh_;
    geometry_msgs::Pose pose_;
    ros::Timer tf_timer_;
    tf::TransformListener listener;

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    Marker makeBox(InteractiveMarker &msg);

    InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg);

    void updateTf(int, const ros::TimerEvent& event);

public:
    InteractivMarkerPublisher(ros::NodeHandle nh);

    void make6DofMarker();
};

#endif