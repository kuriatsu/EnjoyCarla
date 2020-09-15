#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Waypoint.h>

class WaypointPublisher
{
private:
	ros::Publisher pub_waypoint;

public:
	WaypointPublisher();

private:
	void readFile(const std::string &file_name);
    geometry_msgs::Quaternion yawToQuat(float yaw);
};


WaypointPublisher::WaypointPublisher()
{
	ros::NodeHandle n;

	pub_waypoint = n.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", 1, true);

	// std::string file_name = "/home/mad-carla/share/Town05_waypoint_short_demo_1.csv";
    std::string file_name;
	n.getParam("file_name", file_name);
	// std::cout << file_name << std::endl;
    // ROS_INFO(file_name);
	readFile(file_name);

}


void WaypointPublisher::readFile(const std::string &file_name)
{
	std::string line_buf;
	std::ifstream ifs(file_name);

    autoware_msgs::LaneArray out_lane_array;
    autoware_msgs::Lane out_lane;
	autoware_msgs::Waypoint in_waypoint;

	if(!ifs == 2)
	{
		ROS_ERROR("Cannot Open File!!!");
		return;
	}
	ROS_INFO("read file");

	std::getline(ifs, line_buf);
	while (std::getline(ifs, line_buf))
	{
        std::istringstream stream(line_buf);
        std::vector<std::string> list_buf;
        std::string value;

		while (std::getline(stream, value, ','))
		{
			list_buf.emplace_back(value);
		}

		in_waypoint.pose.pose.position.x = std::stof(list_buf.at(4));
		in_waypoint.pose.pose.position.y = std::stof(list_buf.at(5));
		in_waypoint.pose.pose.position.z = std::stof(list_buf.at(6));
        in_waypoint.pose.pose.orientation = yawToQuat(std::stof(list_buf.at(7)));
		out_lane.waypoints.emplace_back(in_waypoint);
	}
    out_lane.header.stamp = ros::Time::now();
    out_lane.header.frame_id = "map";
    out_lane.lane_id = 1;
    out_lane_array.lanes.emplace_back(out_lane);
	pub_waypoint.publish(out_lane_array);
}


geometry_msgs::Quaternion WaypointPublisher::yawToQuat(const float yaw)
{
    tf::Quaternion tf_quat = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion msg_quat;
    quaternionTFToMsg(tf_quat, msg_quat);
    return msg_quat;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "carla_waypoint_publisher_node");

	ROS_INFO("initializing waypoint_publisher");

	WaypointPublisher waypoint_publisher;

	ros::spin();
	return 0;
}
