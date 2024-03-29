#include "ackerman.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include "audi.h"

Ackerman::Ackerman(){
    platformType_ = pfms::PlatformType::ACKERMAN;
    MAX_BRAKE_TORQUE = 8000.0; //Nm
    DEFAULT_THROTTLE = 0.1; //m/s
    i_ = 1;
    brake_ = 0.0;
    steering_ = 0.0;
    throttle_ = 0.0;
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    //uses Audi library which 'automatically' computes values for distance, time and etimated goal pose and if the goal can be reached by Ackerman.
    Audi audi;
    if (audi.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose)){
        return true;
    }
    else {
        return false;
    }
}

bool Ackerman::reachGoal(void){
    //Initialise (or reset) conditions for new reach goal command
    unsigned long repeats = 1;
    throttle_ = 0.1;
    int state = 1;
    //compute steering using Audi library and store the value in steering_ (private data member)
    double originalDistance;
    Audi audi;
    audi.computeSteering(currentOdo_, goal_, steering_, originalDistance);
    //Display goal point in gazebo sim (rviz)
    unsigned int j=0;
    pfms::geometry_msgs::Goal goal{j++,goal_};
    pfmsConnectorPtr_->send(goal);
    while (true){
        //takes in brake, steering and throttle values and inputs to Ackerman cmd, sending commands through pfmsConnector and reads current odo

        pfms::commands::Ackerman cmd {repeats,brake_,steering_,throttle_};

        pfmsConnectorPtr_->send(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //wait for a period to allow information to be sent then retrieved

        pfmsConnectorPtr_->read(currentOdo_,platformType_);
        distanceToCurrentGoal_ = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
        
        switch (state){
            case 0:
                brake_ = 0.0;
                steering_ = 0.0;
                throttle_ = 0.0;
                return true;
            case 1: //start driving, until distance to goal is less than 1m, then switch to apply brakes
                if (distanceToCurrentGoal_ < 1.0){
                    state = 2;
                }
                break;
            case 2: //apply brakes until close at goal
                throttle_ = 0;
                brake_ = 3000; //less than max braking, to just gradually slow down
                if (distanceToCurrentGoal_ < goalTolerance_+0.2){
                    state = 3;
                }
                break;
            case 3: //apply max braking torque to come to a complete stop
                brake_ = MAX_BRAKE_TORQUE;
                if(currentOdo_.linear.x <= 0 && currentOdo_.linear.y <= 0){ //check for when velocity is 0 (stopped) before proceeding with next goal or terminating (finishing) program
                    state = 0;
                }
                break;
        }
        
        if (distanceToCurrentGoal_ >= goalTolerance_){ //incremental counter for sending commands (needs increasing sequence counter)
            repeats++;
        }        
    }
}
