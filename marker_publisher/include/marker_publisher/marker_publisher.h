#ifndef MARKER_PUBLISHER_H
#define MARKER_PUBLISHER_H
#include <string>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

enum class Color {BLACK, WHITE, BROWN, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE};

class MarkerPublisher
{
private:
	struct Private;
    ros::Publisher vis_pub;
    visualization_msgs::Marker marker;
	Color color;

    void init(const std::string ns, const Color c, float scale_x, const float scale_y, const float scale_z, const int type);
public:
	MarkerPublisher(const std::string ns, const Color c);

    MarkerPublisher(const std::string ns, const Color c, const float scale_x, const float scale_y);

	void publishVisualizationMarker(const geometry_msgs::PointStamped& point);

	void publishVisualizationMarkerWithColor(const geometry_msgs::PointStamped& point, const Color color);

	void setColor(const Color color);

};

#endif