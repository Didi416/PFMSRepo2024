#include "skidsteer.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

SkidSteer::SkidSteer(){
    platformType_ = pfms::PlatformType::SKIDSTEER; //sets platform type upon construction of object
    velocity_ = 1.0; //linear velocity in m/s
    angularV_ = 1.0; //rotational velocity in rad/s
    i_ = 1; //initial number of repeats for sending commands to platform
    turnLR_ = 0.0; //assigning zero value for initial command messages
    moveFB_ = 0.0; // asssigning zero values for initial command messages
}

bool SkidSteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    //Skidsteer can reach all goals, as platform turns in place then moved forwards to goal (360 degree rotation capabilities).

    //caluculate angular displacement (rotational angle) from the current position of the platform to the goal point
    angularDisp_ = atan2(std::abs(origin.position.y - goal_.y), std::abs(origin.position.x - goal_.x)) - origin.yaw;

    //calculate distance and time and store them in the referenced variables:
    //calculate current distance to goal upon being called, SkidSteer is a simple calculation:
    //pythagorean theorum between current position and goal (hypotenuse)
    distance = sqrt(pow(origin.position.x - goal_.x, 2) + pow(origin.position.y - goal_.y, 2)); 
    originalDistanceToCurrentGoal_ = distance;
    //calculate current time to goal upon being called, SkidSteer is a simple calculation:
    //calculate the straight line time with t=d/s, then calculate the rotational time through t=theta/omega and add them together
    time = distanceToCurrentGoal_/velocity_;
    angularDisp_ = atan2(std::abs(origin.position.y - goal_.y), std::abs(origin.position.x - goal_.x)) - origin.yaw;
    if (angularDisp_ != 0){
        time += std::abs(angularDisp_)/angularV_;
    }
    originalTimetoCurrentGoal_ = time;
    return true;
}

void SkidSteer::drive(unsigned long i, double turnLR, double moveFB){ // custom function to call several times in reachGoal, depending on state of movement
    //takes in turn and move values (angular velocity and velocity) and inputs to SkidSteer cmd, sending commands through pfmsConnector and reads current odo
    pfms::commands::SkidSteer cmd {i,turnLR,moveFB}; 

    pfmsConnectorPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); //wait for a period to allow information to be sent then retrieved

    pfmsConnectorPtr_->read(currentOdo_,platformType_);
}

bool SkidSteer::reachGoal(void){ //reach goal function computes movements of Skidsteer by calculating angle to turn and then distance to move forward once facing goal
    bool goalReached = false;

    turningAngle_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
    //set angular valocity either positive or negative (left or right) and velocity to 1.0 constant
    if (turningAngle_ < 0){
        angularV_ = 1.0;
    }
    else if (turningAngle_ > 0){
        angularV_ = -1.0;
    }
    velocity_ = 1.0;
    //loop through untl goal reached within tolerance
    while (!goalReached){
        if (angularDisp_ > 0){ //check if turning left or right to face goal (left first)
            if (angularDisp_ >= 0.05){ //while angular dispalcement is above 5 degreees
                drive(1, angularV_, 0); //call drive function (reduces repeating lines by calling function) then recalculate angle to turn
                angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                //if angle is less than 0.3 radians, reduce speed so to not over shoot goal angle
                if (angularDisp_ < 0.3){
                    angularV_ = angularV_*0.75;
                }
            }
            if (angularDisp_ <= 0.05 && distanceToGoal() >= 0.4){ //once finished turning, switch to moving forwards, calculated by pythagoras from currentPos to goal
                // angularV_ = 0.5;
                drive(1, 0, velocity_);
                distanceToCurrentGoal_ = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
                // reduce speed when less than 1m from goal
                if (distanceToGoal() < 1){
                    velocity_ = 0.2;
                }
            }
            //goal has been reached within a tolerance
            if (angularDisp_ <= 0.05 && distanceToGoal() <= 0.4){
                velocity_ = 0;
                angularV_ = 0;
                return true;
            }
        }

        else if (angularDisp_ < 0){ // repeat process for turning right
            if (angularDisp_ <= -0.05){
                drive(1, angularV_, 0);
                angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                if (angularDisp_ > 0.3){
                    angularV_ = angularV_*0.75;
                }
            }
            if (angularDisp_ >= -0.05 && distanceToGoal() >= 0.4){
                drive(1, 0, velocity_);
                distanceToCurrentGoal_ = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
                if (distanceToGoal() < 1){
                    velocity_ = 0.2;
                }
            }
            if (angularDisp_ >= -0.05 && distanceToGoal() <= 0.4){
                return true;
            }
        }
    }
    return false;
}
