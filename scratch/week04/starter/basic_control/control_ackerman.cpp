// Helper utility to send comamnd to ugv
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This executable shows the use of the pipes library to send commands
// and receive odometry from the ugv platform

#include "pfms_types.h"
#include "pfmsconnector.h"
#include "audi.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::Ackerman; // we state this here to be able to refer to Ackerman commands instead of the full name

int main(int argc, char *argv[]) {

    // You can use below as starting point to pass a goal for instance, think what you need to do to parse the input
    // if(argc !=3){
    // }
    // else{
    //     atof(argv[2]);
    // }
    // double x=10.0;
    // double y=5.0;
    // double brake = 0.0;
    // double steering = 0.0;
    // double throttle = 0.1;

    unsigned long repeats = 1;
    double brake = 0.0;
    double steering = 10.0;
    double throttle = 0.2;
    // //! Created a pointer to a Pipe 
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>();
    pfms::nav_msgs::Odometry odo; // We will use this to store odometry
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;

    // throttle = 0.2;
    // brake = 0;
    // steering = 0;
    // int state = 0;
    bool finished = false;
    while (!finished){
        pfmsConnectorPtr->read(odo,type);

        pfms::commands::Ackerman cmd {repeats,brake,steering,throttle};
        // std::cout<<"Steering: "<<steering_<<" Throttle: "<<throttle_<<std::endl;

        pfmsConnectorPtr->send(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        pfmsConnectorPtr->read(odo,type);
        repeats++;

        // currentOdo_ = odo;
        if (repeats == 100){
            finished = true;
        }
        // std::cout<<"Current Odo Readings: "<<std::endl;
        // std::cout<<currentOdo_.position.x<<std::endl;
        // std::cout<<currentOdo_.position.y<<std::endl;
        // std::cout<<currentOdo_.yaw<<std::endl;
    }

    //! We can also send a goal to be visualised on rviz with the following command
    // unsigned int j=0;
    // pfms::geometry_msgs::Point pt{x,y};
    // pfms::geometry_msgs::Goal goal{j++,pt};
    // pfmsConnectorPtr->send(goal);

    //! Finally we can show use of Audi Library
    Audi audi;
    //audi.checkOriginToDestination - look at the function in audi.h

   return 0;
}
