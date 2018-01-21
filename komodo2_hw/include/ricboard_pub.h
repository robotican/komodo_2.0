
#ifndef KOMODO2_HW_RICBOARD_PUB_H
#define KOMODO2_HW_RICBOARD_PUB_H

#include <ric_interface/ric_interface.h>
#include <ric_interface/ric_exception.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono/chrono.hpp>


#define RIC_PORT_PARAM "~ric_port"
#define TORSO_JOINT_PARAM "~torso_joint"
#define RIC_PUB_INTERVAL 0.1 //secs
#define RIC_WRITE_INTERVAL 0.05 //secs
#define RIC_DEAD_TIMEOUT 1 //secs
#define MAX_RIC_DISCONNECTIONS 5
#define SERVO_NEUTRAL 1500

class RicboardPub
{
private:

    bool  load_ric_hw_ = true;
    int ric_disconnections_counter_ = 0;
    std::string ric_port_;
    ros::Publisher ric_gps_pub_;
    ros::Publisher ric_ultrasonic_pub_;
    ros::Publisher ric_imu_pub_;

    ros::Timer ric_pub_timer_,
               ric_dead_timer_;
    ric_interface::RicInterface ric_;
    ros::NodeHandle *nh_;
    boost::thread* t;

    ros::Publisher espeak_pub_;

    /* handles */
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::JointHandle> pos_handles_;

    void pubTimerCB(const ros::TimerEvent& event);
    void ricDeadTimerCB(const ros::TimerEvent& event);
    void loop();
    void speakMsg(std::string msg, int sleep_time)
    {
        std_msgs::String speak_msg;
        speak_msg.data = msg;
        espeak_pub_.publish(speak_msg);
        if (sleep_time > 0)
            sleep(sleep_time);
    }

public:
    RicboardPub(ros::NodeHandle &nh);
    void startLoop();
    void stopLoop();

    /* functions for ros controller use */
    void read(const ros::Duration elapsed);
};


#endif //KOMODO2_HW_RICBOARD_PUB_H
