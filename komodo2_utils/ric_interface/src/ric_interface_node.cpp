#include <ros/ros.h>

#include <ric_interface/ric_interface.h>

ric_interface::RicInterface bm;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ric_interface_node");
    ros::NodeHandle nh;

    bm.connect("/dev/armadillo2/RICBOARD");

    ros::Time prev_time = ros::Time::now();
    while (ros::ok())
    {
        bm.loop();
        if (ros::Time::now() - prev_time >= ros::Duration(0.1))
        {
            ric_interface::protocol::servo actu_pkg;
            actu_pkg.cmd =1500;
            bm.writeCmd(actu_pkg, sizeof(ric_interface::protocol::servo), ric_interface::protocol::Type::SERVO);
            prev_time = ros::Time::now();
        }
        ros::spinOnce;
    }
}




