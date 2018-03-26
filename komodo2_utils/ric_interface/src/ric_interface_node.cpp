#include <ros/ros.h>

#include <ric_interface/ric_interface.h>

ric::RicInterface bm;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ric_interface_node");
    ros::NodeHandle nh;

    bm.connect("/dev/komodo2/RICBOARD");

    ros::Time prev_time = ros::Time::now();
    while (ros::ok())
    {
        bm.loop();
        if (ros::Time::now() - prev_time >= ros::Duration(0.1))
        {
            prev_time = ros::Time::now();
        }
        ros::spinOnce;
    }
}




