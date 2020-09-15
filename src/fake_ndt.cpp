# include <ros/ros.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/PoseStamped.h>
# include <geometry_msgs/TwistStamped.h>
# include <tf/transform_broadcaster.h>


class fakeNdt
{
private:
	ros::Publisher pub_pose;
	ros::Publisher pub_estimate_twist;
	ros::Subscriber sub_odom;

	geometry_msgs::PoseStamped ego_pose;

	tf::TransformBroadcaster br;
	ros::Timer pub_timer;

public:
	fakeNdt();

private:
	void subOdomCallback(const nav_msgs::Odometry &in_odom);
	void pubTf(const ros::TimerEvent&);
};


fakeNdt::fakeNdt()
{
	ros::NodeHandle n;
	pub_pose = n.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1);
	pub_estimate_twist = n.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 1);
	sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 1, &fakeNdt::subOdomCallback, this);

	ros::Duration(1).sleep();
	pub_timer = n.createTimer(ros::Duration(0.3), &fakeNdt::pubTf, this);
}


void fakeNdt::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
	geometry_msgs::TwistStamped ego_twist;

	ego_pose.header.stamp = ros::Time::now();
	ego_pose.header.frame_id = "map";
	ego_twist.header.stamp = ros::Time::now();
	ego_twist.header.frame_id = "map";

	ego_pose.pose = in_odom.pose.pose;
	ego_twist.twist = in_odom.twist.twist;

	pub_estimate_twist.publish(ego_twist);
	pub_pose.publish(ego_pose);
	// pubTf(ego_pose.pose);

}


void fakeNdt::pubTf(const ros::TimerEvent&)
{
	tf::Transform transform;
	// transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
	// transform.setRotation(tf::quaternionMsgToTF(in_pose.orientation));
	tf::poseMsgToTF(ego_pose.pose, transform);
	// br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "map", "base_link"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_ndt_node");
	ROS_INFO("initializing");
	fakeNdt carla_helper;
	ros::spin();
	return 0;
}
