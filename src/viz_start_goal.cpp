#include <ros/ros.h>
#include <jsk_rviz_plugins/PictogramArray.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class VizStartGoal
{
private:
	ros::Publisher pub_pictogram;
	ros::Timer pub_timer;
	jsk_rviz_plugins::PictogramArray m_pictogram_array;

public:
	VizStartGoal();

private:
	void timerCb(const ros::TimerEvent&);
	geometry_msgs::Pose gerPoseFromParam(const std::string &string);
    void initPictogram(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal);
};


VizStartGoal::VizStartGoal()
{
	ros::NodeHandle nh;
	pub_pictogram = nh.advertise<jsk_rviz_plugins::PictogramArray>("/start_goal_marker", 1);

	std::string goal, start;
	nh.getParam("/viz_start_goal/start", start);
	nh.getParam("/viz_start_goal/goal", goal);
    std::cout << gerPoseFromParam(start) << "," << gerPoseFromParam(goal) << std::endl;
	initPictogram(gerPoseFromParam(start), gerPoseFromParam(goal));

	ros::Duration(1).sleep();
	pub_timer = nh.createTimer(ros::Duration(1), &VizStartGoal::timerCb, this);
}


geometry_msgs::Pose VizStartGoal::gerPoseFromParam(const std::string &string)
{
	std::stringstream ss(string);
	std::string item;
	std::vector<std::string> buf_list;
	geometry_msgs::Pose out_pose;
	tf2::Quaternion buf_quat;

	while (getline(ss, item, ' '))
	{
		buf_list.emplace_back(item);
	}

	if (buf_list.size() == 4)
	{
		out_pose.position.x = std::stof(buf_list.at(0));
		out_pose.position.y = -std::stof(buf_list.at(1));
		out_pose.position.z = std::stof(buf_list.at(2));
		buf_quat.setRPY(M_PI*0.5, 0.0, std::stof(buf_list.at(3)) * M_PI /(180.0));
		out_pose.orientation = tf2::toMsg(buf_quat);
	}

	return out_pose;
}


void VizStartGoal::initPictogram(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal)
{
	jsk_rviz_plugins::Pictogram pictogram;
	tf2::Quaternion orig_tf_quat, rot_tf_quat, new_tf_quat;
	geometry_msgs::Quaternion new_geo_quat;

	pictogram.header.frame_id = "/map";
	pictogram.action = jsk_rviz_plugins::Pictogram::ADD;
	pictogram.ttl = 1.0;
	pictogram.color.a = 1.0;
	// rot_tf_quat.setRPY(0.0, M_PI*0.5, 0.0);

	// start icons
	pictogram.pose = start;
	pictogram.color.g = 1.0;

    pictogram.pose.orientation.x = 0;
    pictogram.pose.orientation.y = 0;
	pictogram.pose.orientation.w = 1;
	pictogram.pose.orientation.z = 1;
	pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
	pictogram.character = "fa-circle";
	pictogram.size = 10.0;
	m_pictogram_array.pictograms.push_back(pictogram);

    tf2::fromMsg(pictogram.pose.orientation, orig_tf_quat);
	tf2::fromMsg(start.orientation, rot_tf_quat);
    new_tf_quat = rot_tf_quat * orig_tf_quat;
    pictogram.pose.orientation = tf2::toMsg(new_tf_quat);
    pictogram.pose.position.z += 2;
	pictogram.mode = jsk_rviz_plugins::Pictogram::STRING_MODE;
	pictogram.character = "START";
	pictogram.size = 5.0;
	m_pictogram_array.pictograms.push_back(pictogram);

	// goal icons
	pictogram.pose = goal;
	pictogram.color.r = 1.0;

    pictogram.pose.orientation.x = 0;
    pictogram.pose.orientation.y = 0;
	pictogram.pose.orientation.w = 1;
    pictogram.pose.orientation.z = 1;
	pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
	pictogram.character = "fa-circle";
	pictogram.size = 10.0;
	m_pictogram_array.pictograms.push_back(pictogram);

    tf2::fromMsg(pictogram.pose.orientation, orig_tf_quat);
    tf2::fromMsg(goal.orientation, rot_tf_quat);
    new_tf_quat = rot_tf_quat * orig_tf_quat;
    pictogram.pose.orientation = tf2::toMsg(new_tf_quat);
    pictogram.pose.position.z += 2;
	pictogram.mode = jsk_rviz_plugins::Pictogram::STRING_MODE;
	pictogram.character = "GOAL";
	pictogram.size = 5.0;
	m_pictogram_array.pictograms.push_back(pictogram);
}


void VizStartGoal::timerCb(const ros::TimerEvent &)
{
	m_pictogram_array.header.stamp = ros::Time::now();
	m_pictogram_array.header.frame_id = "/map";
	pub_pictogram.publish(m_pictogram_array);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "viz_start_goal_node");
	VizStartGoal viz_start_goal;
	ros::spin();
	return 0;
}
