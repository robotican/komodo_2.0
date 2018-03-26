
#include <ric_interface/ric_interface.h>


namespace ric
{
    RicInterface::RicInterface()
    {

    }

    /* open connection to serial port                */
    /* if conncetion fails, exception will be thrown */
    void RicInterface::connect(std::string port)
    {
        comm_.connect(port, 500000);
    }

    void RicInterface::clearBuffer()
    {
        memset(pkg_buff_, 0, protocol::MAX_PKG_SIZE);
    }

    void RicInterface::loop()
    {
        sendKeepAlive();
        readAndHandlePkg();
        checkKeepAliveFromRic();
    }

    void RicInterface::checkKeepAliveFromRic()
    {
        get_keepalive_timer_.startTimer(GET_KA_TIMEOUT);
        if (get_keepalive_timer_.isFinished())
        {
            if (got_keepalive_) //connected
            {
                is_board_alive_ = true;
                got_keepalive_ = false;
                //fprintf(stderr,"board alive ! \n");
            }
            else
            {
                is_board_alive_ = false;
                //fprintf(stderr,"board dead ! \n");
            }
            get_keepalive_timer_.reset();
        }
    }

    std::string RicInterface::compType2String(const protocol::Type comp_type)
    {
        switch (comp_type)
        {
            case protocol::Type::KEEP_ALIVE:
                return "KEEP_ALIVE";
            case protocol::Type::LOGGER:
                return "LOGGER";
            case protocol::Type::ERROR:
                return "ERROR";
            case protocol::Type::ULTRASONIC:
                return "ULTRASONIC";
            case protocol::Type::IMU:
                return "IMU";
            case protocol::Type::GPS:
                return "GPS";
        }
        return NULL;
    }

    std::string RicInterface::errCode2String(const protocol::ErrCode err_code)
    {
        switch (err_code)
        {
            case protocol::ErrCode::INIT:
                return "initialization failed";
            case protocol::ErrCode::READ:
                return "reading failed";
            case protocol::ErrCode::CALIB:
                return "can't find calibration data";
        }
        return NULL;
    }

    void RicInterface::readAndHandlePkg()
    {
        int pkg_type = comm_.read(pkg_buff_);
        if (pkg_type > 0)
        {
            //fprintf(stderr, "pkg_type: %d\n", pkg_type);
            switch (pkg_type)
            {
                case (uint8_t) protocol::Type::KEEP_ALIVE:
                {
                    protocol::keepalive ka_pkg;
                    Communicator::fromBytes(pkg_buff_, sizeof(protocol::keepalive), ka_pkg);
                    if (ka_pkg.type == (uint8_t)protocol::Type::KEEP_ALIVE)
                    {
                        //fprintf(stderr, "got keep alive\n");
                        got_keepalive_ = true;
                    }
                    break;
                }
                case (uint8_t) protocol::Type::LOGGER:
                {
                    protocol::logger logger_pkg;
                    Communicator::fromBytes(pkg_buff_, sizeof(protocol::logger), logger_pkg);
                    if (logger_pkg.type == (uint8_t)protocol::Type::LOGGER)
                    {
                        got_new_logger_msg_ = true;
                        sensors_state_.logger = logger_pkg;
                        //fprintf(stderr, "logger msg: %s, code: %d\n", logger_pkg.msg, logger_pkg.value);
                    }

                    break;
                }
                case (uint8_t) protocol::Type::ERROR:
                {
                    protocol::error err_pkg;
                    Communicator::fromBytes(pkg_buff_, sizeof(protocol::error), err_pkg);
                    if (err_pkg.type == (uint8_t)protocol::Type::ERROR)
                    {
                        got_new_error_msg_ = true;
                        sensors_state_.error = err_pkg;
                        //fprintf(stderr, "error comp: %i, code: %i\n", sensors_state_.error.comp_type, sensors_state_.error.code);
                    }

                    break;
                }
                case (int)protocol::Type::ULTRASONIC:
                {
                    protocol::ultrasonic urf_pkg;
                    Communicator::fromBytes(pkg_buff_, sizeof(protocol::ultrasonic), urf_pkg);
                    if (urf_pkg.type == (uint8_t)protocol::Type::ULTRASONIC)
                    {
                        switch (urf_pkg.id)
                        {
			    case protocol::ultrasonic::ID_RIGHT:
                            {
                                sensors_state_.urf_right = urf_pkg;
                                break;
                            }

                            case protocol::ultrasonic::ID_REAR:
                            {
                                sensors_state_.urf_rear = urf_pkg;
                                break;
                            }
                            
                            case protocol::ultrasonic::ID_LEFT:
                            {
                                sensors_state_.urf_left = urf_pkg;
                                break;
                            }
                        }
                        //printf(stderr, "ultrasonic: %d\n", urf_pkg.distance_mm);
                    }
                    break;
                }
                case (int)protocol::Type::IMU:
                {
                    protocol::imu imu_pkg;
                    Communicator::fromBytes(pkg_buff_, sizeof(protocol::imu), imu_pkg);
                    if (imu_pkg.type == (uint8_t)protocol::Type::IMU)
                    {
                        sensors_state_.imu = imu_pkg;
                        /*fprintf(stderr, "imu:\troll: %f,\tpitch: %f,\tyaw: %f \n",
                                sensors_state_.imu.roll_rad * 180 / M_PI,
                                sensors_state_.imu.pitch_rad * 180 / M_PI,
                                sensors_state_.imu.yaw_rad * 180 / M_PI);
                        fprintf(stderr, "imu:\taccl_x: %f,\taccl_y: %f,\taccl_z: %f \n",
                                sensors_state_.imu.accl_x_rad * 180 / M_PI,
                                sensors_state_.imu.accl_y_rad * 180 / M_PI,
                                sensors_state_.imu.accl_z_rad * 180 / M_PI);
                        fprintf(stderr, "imu:\tgyro_x: %f,\tgyro_y: %f,\tgyro_z: %f \n",
                                sensors_state_.imu.gyro_x_rad * 180 / M_PI,
                                sensors_state_.imu.gyro_y_rad * 180 / M_PI,
                                sensors_state_.imu.gyro_z_rad * 180 / M_PI);
                        fprintf(stderr, "imu:\tmag_x: %f,\tmag_y: %f,\tmag_z: %f \n",
                                sensors_state_.imu.mag_x_rad * 180 / M_PI,
                                sensors_state_.imu.mag_y_rad * 180 / M_PI,
                                sensors_state_.imu.mag_z_rad * 180 / M_PI);*/
                    }
                    break;
                }
                case (int)protocol::Type::GPS:
                {
                    protocol::gps gps_pkg;
                    Communicator::fromBytes(pkg_buff_, sizeof(protocol::gps), gps_pkg);
                    if (gps_pkg.type == (uint8_t)protocol::Type::GPS)
                    {
                        sensors_state_.gps = gps_pkg;
                        //fprintf(stderr,"gps lat: %f, lon: %f\n", sensors_state_.gps.lat, sensors_state_.gps.lon);
                    }
                    break;
                }
            }
            clearBuffer();
        }
    }

    /* read logger msg if available, and mark read */
    bool RicInterface::readLoggerMsg(std::string &msg, int32_t &value)
    {
        if (got_new_logger_msg_)
        {
            got_new_logger_msg_ = false; //clear msg flag
            msg = sensors_state_.logger.msg;
            value = sensors_state_.logger.value;
            return true;
        }
        return false;
    }

    /* read error msg if available, and mark read */
    bool RicInterface::readErrorMsg(protocol::error &error)
    {
        if (got_new_error_msg_)
        {
            got_new_error_msg_ = false; //clear msg flag
            error = sensors_state_.error;
            return true;
        }
        return false;
    }

    void RicInterface::sendKeepAlive()
    {
        send_keepalive_timer_.startTimer(SEND_KA_TIMEOUT);
        if (send_keepalive_timer_.isFinished())
        {
            //puts("sending ka");
            protocol::keepalive ka_pkg;
            comm_.write(ka_pkg, sizeof(protocol::keepalive));
            send_keepalive_timer_.reset();
        }
    }

}
