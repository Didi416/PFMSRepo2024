// Helper utility to send comamnd to ugv
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This executable shows the use of the pipes library to send commands
// and receive odometry from the ugv platform

#include "pfmsconnector.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::Ackerman; // we state this here to be able to refer to UGV instead of the full name

int main(int argc, char *argv[]) {

    int repeats = 100;
    double brake = 0.0;
    double steering = 0.0;
    double throttle = 0.2;

    if(argc !=5){
         cout << " Not arguments given on command line." << endl;
         cout << " using defaults for: " << argv[0] << "<repeats>" << repeats << 
            " <brake>" << brake << " <steering>" <<
            steering << "<throttle>" << throttle << endl;
    }
    else{
        repeats = atoi(argv[1]);
        brake = atof(argv[2]);
        steering = atof(argv[3]);
        throttle = atof(argv[4]);
        cout << "Using for: " << argv[0] << " <repeats>" << repeats << 
            " <brake>" << brake << " <steering>" <<
            steering << " <throttle>" << throttle << endl;
    }

    //! Created a pointer to a Pipe 
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>();
    pfms::nav_msgs::Odometry odo;
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;

    unsigned long i = 0;
    /* produce messages */
    for(i = 0; i < repeats; i ++) {        
        Ackerman cmd {
                    i,
                    brake,
                    steering,
                    throttle,
                 };
        pfmsConnectorPtr->send(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        bool OK  =  pfmsConnectorPtr->read(odo,type);

        if(OK){
            std::cout << 
                i << " " <<
                odo.time << " " <<
                odo.position.x << " " <<
                odo.position.y << " " <<
                odo.yaw << " " <<
                odo.linear.x << " " <<
                odo.linear.y << std::endl;
        }
        std::this_thread::sleep_for (std::chrono::milliseconds(10));        
    }
    
   return 0;
}
