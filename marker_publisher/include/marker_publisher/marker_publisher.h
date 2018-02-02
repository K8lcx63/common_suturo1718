#ifndef MARKER_PUBLISHER_H
#define MARKER_PUBLISHER_H
#include <string>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

enum class Color {BLACK, WHITE, BROWN, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE};

class MarkerPublisher
{
private:
    ros::Publisher vis_pub;
    visualization_msgs::Marker marker;

    void init(const std::string ns, const Color c, float scale_x, const float scale_y, const float scale_z, const int type);
public:
	MarkerPublisher(const std::string ns, const Color c);

    MarkerPublisher(const std::string ns, const Color c, const float scale_x, const float scale_y);

	void publishVisualizationMarker(geometry_msgs::PointStamped point);
};

#endif