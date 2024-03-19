#include "controller.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Controller::Controller(){

}

bool Controller::setGoal(pfms::geometry_msgs::Point goal){ //Updates variables associated with timeToGoal and distanceToGoal functions
    goal_ = goal;
    double distance = 0;
    double time = 0;
    pfms::nav_msgs::Odometry estimatedGoalPose;

    currentOdo_ = getOdometry();
    pfms::nav_msgs::Odometry origin = currentOdo_;
    // std::cout<<"Initial Odo Readings: "<<std::endl;
    // std::cout<<currentOdo_.position.x<<std::endl;
    // std::cout<<currentOdo_.position.y<<std::endl;
    // std::cout<<currentOdo_.yaw<<std::endl;
    if (!checkOriginToDestination(origin, goal_, distance, time, estimatedGoalPose)){
        return false;
    }
    else{
        return true;
    }
}

pfms::PlatformType Controller::getPlatformType(void){
    pfms::PlatformType platform = platformType_; //accesses protected value in Controller class
    return platform;
}

double Controller::distanceTravelled(void){
    double totalDistance = 0;

    return totalDistance;
}

double Controller::timeInMotion(void){
    double totalTime = 0;

    return totalTime;
}

bool Controller::setTolerance(double tolerance){
    return true;
}
