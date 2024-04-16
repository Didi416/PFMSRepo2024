#include "controller.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Controller::Controller(){

}
Controller::~Controller(){

}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    goals_ = goals;
    for (auto goal:goals_){
        getOdometry();
        std::cout<<"Current Odo Readings: "<<std::endl;
        std::cout<<currentOdo_.position.x<<std::endl;
        std::cout<<currentOdo_.position.y<<std::endl;
        std::cout<<currentOdo_.position.z<<std::endl;
        std::cout<<currentOdo_.yaw<<std::endl;
        checkOriginToDestination(currentOdo_, goal, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
    }
    return true;
}

double Controller::distanceToGoal(void){
    return 0.0;
}

double Controller::timeToGoal(void){
    return 0.0;
}

double Controller::distanceTravelled(void){
    return 0.0;
}

double Controller::timeTravelled(void){
    return 0.0;
}

pfms::PlatformType Controller::getPlatformType(void){
    return platformType_;
}

pfms::PlatformStatus Controller::status(void){
    return platformStatus_;
}

bool Controller::setTolerance(double tolerance){
    goalTolerance_ = tolerance;
    return true;
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){
    pfmsConnectorPtr_->read(currentOdo_,platformType_);
    return currentOdo_;
}

std::vector<pfms::geometry_msgs::Point> Controller::getObstacles(void){
    std::vector<pfms::geometry_msgs::Point> obstacles;
    return obstacles;
}