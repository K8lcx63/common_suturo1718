#ifndef MARKER_PUBLISHER_H
#define MARKER_PUBLISHER_H
#include <geometry_msgs/PointStamped.h>
#include <string>

enum class Color {BLACK, WHITE, BROWN, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE};

class MarkerPublisher
{
private:
    ros::Publisher vis_pub;
    Color color;
    std::string name_space;

public:
	MarkerPublisher(const std::string ns, const Color c);

	void publishVisualizationMarker(geometry_msgs::PointStamped point);
};

#endif