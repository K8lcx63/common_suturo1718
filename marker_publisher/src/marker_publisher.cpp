#include <ros/ros.h>
#include <marker_publisher/marker_publisher.h>

struct MarkerPublisher::Private {
    static void copyDataFromPoint(visualization_msgs::Marker& marker, const geometry_msgs::PointStamped& point) {
        marker.header.frame_id = point.header.frame_id;
        marker.header.stamp = ros::Time();
        marker.pose.position.x = point.point.x;
        marker.pose.position.y = point.point.y;
        marker.pose.position.z = point.point.z;
    }

    static void determineMarkerColor(visualization_msgs::Marker& marker, const Color& color) {
        switch(color) {
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
    }
};

MarkerPublisher::MarkerPublisher(const std::string ns, const Color c)
{
    ros::NodeHandle nh;
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    init(ns, c, 0.1, 0.1, 0.1, visualization_msgs::Marker::SPHERE);
}

MarkerPublisher::MarkerPublisher(const std::string ns, const Color c, const float scale_x, const float scale_y)
{
    ros::NodeHandle nh;
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    init(ns, c, scale_x, scale_y, 0.05, visualization_msgs::Marker::CUBE);
}

void MarkerPublisher::init(const std::string ns, const Color c, float scale_x, const float scale_y, const float scale_z, const int type)
{
    marker.id = 0;
    marker.ns = ns;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 1.0;
    Private::determineMarkerColor(marker, c);
}

void MarkerPublisher::publishVisualizationMarker(const geometry_msgs::PointStamped& point) {
    Private::copyDataFromPoint(marker, point);
    vis_pub.publish(marker);
}

void MarkerPublisher::publishVisualizationMarkerWithColor(const geometry_msgs::PointStamped &point,
                                                          const Color color) {
    Private::copyDataFromPoint(marker, point);
    Private::determineMarkerColor(marker, color);
    vis_pub.publish(marker);
}

void MarkerPublisher::setColor(const Color color) {
    Private::determineMarkerColor(marker, color);
}