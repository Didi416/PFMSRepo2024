#include "controller.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Controller::Controller(){

}

Controller::~Controller(){

}

bool Controller::setGoal(pfms::geometry_msgs::Point goal){ //Updates variables associated with timeToGoal and distanceToGoal functions
    goal_ = goal;
    pfms::nav_msgs::Odometry estimatedGoalPose;
    currentOdo_ = getOdometry();
    std::cout<<"Current Odo Readings: "<<std::endl;
    std::cout<<currentOdo_.position.x<<std::endl;
    std::cout<<currentOdo_.position.y<<std::endl;
    std::cout<<currentOdo_.yaw<<std::endl;
    if (checkOriginToDestination(currentOdo_, goal_, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose)){
        return true;
    }
    else{
        return false;
    }
}

pfms::PlatformType Controller::getPlatformType(void){
    pfms::PlatformType platform = platformType_; //accesses protected value in Controller class
    return platform;
}

double Controller::distanceTravelled(void){
    totalDistance_ += distanceToGoal();
    return totalDistance_;
}

double Controller::timeInMotion(void){
    totalTime_ += timeToGoal();
    return totalTime_;
}

bool Controller::setTolerance(double tolerance){
    goalTolerance_ = tolerance;
    return true;
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){
    pfmsConnectorPtr_->read(currentOdo_,platformType_);
    return currentOdo_;
}