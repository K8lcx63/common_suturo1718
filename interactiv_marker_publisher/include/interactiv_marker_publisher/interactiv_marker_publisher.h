#ifndef INTERACTIV_MARKER_PUBLISHER_H
#define INTERACTIV_MARKER_PUBLISHER_H
#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>

enum class Color {BLACK, WHITE, BROWN, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE};

class InteractivMarkerPublisher
{
private:
    interactive_markers::InteractiveMarkerServer server;

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

public:
    InteractivMarkerPublisher();

    void addInteractivMarker(const std::string name, const std::string meshPath);

	void addInteractivMarker(const std::string name, const float scaleX, const float scaleY, const Color color);
};

#endif