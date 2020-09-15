#include <ros/ros.h>
#include <jsk_rviz_plugins/Pictogram.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;

    ros::Publisher pub_pictgram = n.advertise<jsk_rviz_plugins::Pictogram>("pictogram", 5);
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        jsk_rviz_plugins::Pictogram pictogram;
        pictogram.header.frame_id = "base_link";
        pictogram.header.stamp = ros::Time::now();
        pictogram.action = jsk_rviz_plugins::Pictogram::JUMP;
        pictogram.pose.orientation.x = 0.0;
        pictogram.pose.orientation.y = 1.0;
        pictogram.pose.orientation.z = 0.0;
        pictogram.pose.orientation.w = -1.0;
        pictogram.color.r = 0.4;
        pictogram.color.g = 0.5;
        pictogram.color.a = 1.0;
        pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
        pictogram.character = "fa-angle-double-down";
        pictogram.speed = 10;
        pictogram.ttl = 1000;
        pictogram.size = 0.5;
        pub_pictgram.publish(pictogram);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
