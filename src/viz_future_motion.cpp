#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>

class VizFutureMotion
{
private:
	ros::Subscriber sub_waypoiints;
	ros::Publisher pub_marker;

public:
	VizFutureMotion();

private:
	void subWaypoiintsCb();
};

VizFutureMotion::VizFutureMotion()
{
	ros::NodeHandle nh;
	pub_marker = nh.advertise<visualization_msgs::Marker>("/future_motion", 1);
	sub_waypoiints = nh.subscribe("final_waypoint", &VizFutureMotion::subWaypoiintsCb, this);
}


void VizFutureMotion::subWaypoiintsCb()
{
	visualization_msgs::Marker marker;
	std_msgs::ColorRGBA color;

	marker.header.stamp = ros::Time::now()
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0.1);
	marker.scale = geometry_msgs::Vector3{2.0, 0.0, 0.0};

	for (const auto &waypoint: in_waypoint_list)
	{
		marker.points.emplace_back(waypoint.pose.position);
		if (waypoint.velocity > 5.0)
		{
			color.r = 0.0;
			color.g = 1.0;
			color.b = 0.0;
			color.a = 1.0;
		}
		else if (waypoint.velocity > 2.0)
		{
			color.r = 0.5;
			color.g = 0.5;
			color.b = 0.0;
			color.a = 1.0;
		}
		else
		{
			color.r = 1.0;
			color.g = 0.0;
			color.b = 0.0;
			color.a = 1.0;
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
