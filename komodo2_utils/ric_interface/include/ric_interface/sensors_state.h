//
// Created by armadillo2 on 27/11/17.
//

#ifndef RIC_INTERFACE_SENSORSSTATE_H
#define RIC_INTERFACE_SENSORSSTATE_H

#include "protocol.h"

namespace ric
{
    struct sensors_state
    {
        protocol::logger logger;
        protocol::error error;
        protocol::ultrasonic urf_rear, urf_right, urf_left;
        protocol::imu imu;
        protocol::potentio potentio;
        protocol::gps gps;
        protocol::emergency_alarm emrgcy_alarm;
    };
}


#endif //RIC_INTERFACE_SENSORSSTATE_H
