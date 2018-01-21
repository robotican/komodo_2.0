
#include "ricboard_pub.h"

RicboardPub::RicboardPub(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* get ric params */
    ros::param::get("~load_ric_hw", load_ric_hw_);

    if (load_ric_hw_)
    {
        if (!ros::param::get(RIC_PORT_PARAM, ric_port_))
        {
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: %s param is missing on param server. make sure that you load this param exist in ricboard_config.yaml "
                              "and that your launch includes this param file. shutting down...", RIC_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        try{
            ric_.connect(ric_port_);
            ROS_INFO("[armadillo2_hw/ricboard_pub]: ricboard port opened successfully \nport name: %s \nbaudrate: 115200", ric_port_.c_str());
        }catch (ric_interface::ConnectionExeption e) {
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: can't open ricboard port. make sure that ricboard is connected. shutting down...");
            ros::shutdown();
            exit(1);
        }

        /* ric publishers */
        ric_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("GPS/fix", 10);
        ric_ultrasonic_pub_ = nh.advertise<sensor_msgs::Range>("URF/front", 10);
        ric_imu_pub_ = nh.advertise<sensor_msgs::Imu>("IMU", 10);

        ric_pub_timer_ = nh.createTimer(ros::Duration(RIC_PUB_INTERVAL), &RicboardPub::pubTimerCB, this);
        ric_dead_timer_ = nh.createTimer(ros::Duration(RIC_DEAD_TIMEOUT), &RicboardPub::ricDeadTimerCB, this);
        ROS_INFO("[armadillo2_hw/ricboard_pub]: ricboard is up");
        espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        /*speakMsg("rik board is up", 1); */
    }
    else
        ROS_WARN("[armadillo2_hw/ricboard_pub]: ric hardware is disabled");
}

void RicboardPub::startLoop()
{
    if (!load_ric_hw_)
        return;
    t = new boost::thread(boost::bind(&RicboardPub::loop, this));
}

void RicboardPub::stopLoop()
{
    if (!load_ric_hw_)
        return;
    t->interrupt();
}

void RicboardPub::loop()
{
    if (!load_ric_hw_)
        return;
    while (ros::ok() && !t->interruption_requested())
    {
        ric_.loop();
        if (ric_.isBoardAlive())
        {
            ric_interface::protocol::error err_msg;
            std::string logger_msg;
            int32_t logger_val;
            ric_disconnections_counter_ = 0;
            ric_dead_timer_.stop();
            ric_pub_timer_.start();

            /* if emergecy pin disconnected, shutdown. ric will also kill torso */
            if (ric_.getSensorsState().emrgcy_alarm.is_on)
            {
                speakMsg("emergency, shutting down", 1);
                ROS_ERROR("[armadillo2_hw/ricboard_pub]: EMERGENCY PIN DISCONNECTED, shutting down...");
                ros::shutdown();
                exit(EXIT_FAILURE);
            }
            if (ric_.readLoggerMsg(logger_msg, logger_val))
                ROS_INFO("[armadillo2_hw/ricboard_pub]: ric logger is saying: '%s', value: %i", logger_msg.c_str(), logger_val);
            if (ric_.readErrorMsg(err_msg))
            {
                std::string comp_name = ric_interface::RicInterface::compType2String((ric_interface::protocol::Type)err_msg.comp_type);
                ROS_WARN("[armadillo2_hw/ricboard_pub]: ric detected error in %s", comp_name.c_str());
            }
        }
        else
        {
            ric_dead_timer_.start();
            ric_pub_timer_.stop();
        }
    }
}

void RicboardPub::ricDeadTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_)
        return;
    ric_disconnections_counter_++;
    if (ric_disconnections_counter_ >= MAX_RIC_DISCONNECTIONS)
    {
        speakMsg("rik board disconnected, shutting down", 1);
        ROS_ERROR("[armadillo2_hw/ricboard_pub]: ricboard disconnected. shutting down...");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
}

void RicboardPub::pubTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_ || !ric_.isBoardAlive())
        return;

    ric_interface::sensors_state sensors = ric_.getSensorsState();

    /* publish ultrasonic */
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = "front_urf_link";
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = sensors.ultrasonic.distance_mm / 1000.0;
    range_msg.field_of_view = 0.7f;
    ric_ultrasonic_pub_.publish(range_msg);

    /* publish imu */
    sensor_msgs::Imu imu_msg;

    tf::Quaternion orientation_q = tf::createQuaternionFromRPY(sensors.imu.roll_rad,
                                                               sensors.imu.pitch_rad,
                                                               sensors.imu.yaw_rad);
    imu_msg.orientation.x = orientation_q.x();
    imu_msg.orientation.y = orientation_q.y();
    imu_msg.orientation.z = orientation_q.z();
    imu_msg.orientation.w = orientation_q.w();
    imu_msg.angular_velocity.x = sensors.imu.gyro_x_rad;
    imu_msg.angular_velocity.y = sensors.imu.gyro_y_rad;
    imu_msg.angular_velocity.z = sensors.imu.gyro_z_rad;
    imu_msg.linear_acceleration.x = sensors.imu.accl_x_rad;
    imu_msg.linear_acceleration.y = sensors.imu.accl_y_rad;
    imu_msg.linear_acceleration.z = sensors.imu.accl_z_rad;
    imu_msg.header.stamp = ros::Time::now();
    ric_imu_pub_.publish(imu_msg);

    /* publish gps if data is available */
    if (sensors.gps.satellites > 0)
    {
        sensor_msgs::NavSatFix gps_msg;
        sensor_msgs::NavSatStatus gps_status;
        gps_status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        gps_status.status = sensor_msgs::NavSatStatus::SERVICE_GPS;

        gps_msg.latitude = sensors.gps.lat;
        gps_msg.longitude = sensors.gps.lon;
        gps_msg.status = gps_status;
        gps_msg.header.stamp = ros::Time::now();

        ric_gps_pub_.publish(gps_msg);
    }
}

void RicboardPub::read(const ros::Duration elapsed)
{
    if (!load_ric_hw_ || !ric_.isBoardAlive())
        return;

    /* update robot state according to ric sensor for controller use */
    ric_interface::sensors_state sensors = ric_.getSensorsState();
}


