#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <autoware_msgs/Lane.h>

class VizFutureMotion
{
private:
	ros::Subscriber sub_waypoiints;
	ros::Publisher pub_marker;

public:
	VizFutureMotion();

private:
	void subWaypointsCb(const autoware_msgs::LaneConstPtr &in_lane);
};

VizFutureMotion::VizFutureMotion()
{
	ros::NodeHandle nh;
	pub_marker = nh.advertise<visualization_msgs::Marker>("/future_motion", 1);
	sub_waypoiints = nh.subscribe("final_waypoints", 1, &VizFutureMotion::subWaypointsCb, this);
}


void VizFutureMotion::subWaypointsCb(const autoware_msgs::LaneConstPtr &in_lane)
{
	visualization_msgs::Marker marker;
	std_msgs::ColorRGBA color;

	marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "/map";
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0.1);
    marker.scale.x = 3.0;

	for (const autoware_msgs::Waypoint &waypoint: in_lane->waypoints)
	{
        geometry_msgs::Point marker_position = waypoint.pose.pose.position;
        marker_position.z -= 0.5;
		marker.points.emplace_back(marker_position);
		if (waypoint.twist.twist.linear.x > 5.0)
		{
			color.r = 0.0;
			color.g = 1.0;
			color.b = 0.0;
			color.a = 0.3;
		}
		else if (waypoint.twist.twist.linear.x > 2.0)
		{
			color.r = 0.5;
			color.g = 0.5;
			color.b = 0.0;
			color.a = 0.3;
		}
		else
		{
			color.r = 1.0;
			color.g = 0.0;
			color.b = 0.0;
			color.a = 0.3;
		}
		marker.colors.emplace_back(color);
	}

	pub_marker.publish(marker);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "viz_future_motion_node");
	VizFutureMotion viz_future_motion;
	ros::spin();
	return 0;
}
