
#include "komodo2_hw.h"

namespace komodo2_hw
{

    ArmadilloHW::ArmadilloHW(ros::NodeHandle &nh) :
            dxl_motors_(nh), battery_(nh), ric_(nh), roboteq_(nh)
    {
        node_handle_ = &nh;

        /* register handles */
        dxl_motors_.registerHandles(joint_state_interface_,
                                    position_interface_,
                                    posvel_interface_);
        ric_.registerHandles(joint_state_interface_,
                             effort_interface_);
        roboteq_.registerHandles(joint_state_interface_,
                                 velocity_interface_);

        /* register interfaces */
        registerInterface(&joint_state_interface_);
        registerInterface(&posvel_interface_);
        registerInterface(&position_interface_);
        registerInterface(&velocity_interface_);
        registerInterface(&effort_interface_);

        prev_time_ = ros::Time::now();

        ric_.startLoop();

        ROS_INFO("[armadillo2_hw]: armadillo hardware interface loaded successfully");
        espeak_pub_ = node_handle_->advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        speakMsg("i am ready", 1);
    }

    void ArmadilloHW::read()
    {
        ros::Duration period = ros::Time::now() - prev_time_;
        dxl_motors_.read();
        roboteq_.read(period);
        ric_.read(period);
    }

    void ArmadilloHW::write()
    {
        ros::Duration period = ros::Time::now() - prev_time_;
        dxl_motors_.write();
        roboteq_.write(period);
        ric_.write(period);
        prev_time_ = ros::Time::now();
    }
}
