#ifndef TEST_HELPER_H
#define TEST_HELPER_H

#include "pfms_types.h"
#include <cmath>

using namespace pfms::nav_msgs;

/////////////////////////////////////////////////////////////////////////////////////
/// \brief populateOdo
/// \param veh - 1 for Audi and 2 for Quad
/// \param x - position x
/// \param y - position y
/// \param yaw - yaw in radians
/// \return assembled odo message (with zero for velocity)
///
Odometry populateOdoUGV(double x, double y, double yaw){
    Odometry odo;
    odo.time=1;//Only for the purposes of this testing do we piggyback on this message and send seq to be 1 or 2
    odo.position.x=x;
    odo.position.y=y;
    odo.position.z=0;
    odo.yaw=yaw;
    odo.linear.x=0;
    odo.linear.y=0;
    return odo;
}

Odometry populateOdoUAV(double x, double y, double z, double yaw){
    Odometry odo;
    odo.time=2;//Only for the purposes of this testing do we piggyback on this message and send seq to be 1 or 2
    odo.position.x=x;
    odo.position.y=y;
    odo.position.z=z;
    odo.yaw=yaw;
    odo.linear.x=0;
    odo.linear.y=0;
    return odo;
}

#endif // TEST_HELPER_H
