#include "controller.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Controller::Controller(){
    distanceTravelled_ = 0;
    startToCurrentGoalDist_ = 0;
}
Controller::~Controller(){

}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    goals_ = goals;
    pfms::nav_msgs::Odometry origin = getOdometry();
    for (auto goal:goals_){
        std::cout<<"Current Odo Readings: "<<std::endl;
        std::cout<<origin.position.x<<std::endl;
        std::cout<<origin.position.y<<std::endl;
        std::cout<<origin.position.z<<std::endl;
        std::cout<<origin.yaw<<std::endl;
        bool check = checkOriginToDestination(origin, goal, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
        std::cout<<"Check: "<<check<<std::endl;
        origin = estimatedGoalPose_;
    }
    return true;
}

double Controller::distanceToGoal(void){   
    checkOriginToDestination(getOdometry(), currentGoal_, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
    return distanceToCurrentGoal_; //protected variable so can be accessed from other functions in classes( base and derived)
}

double Controller::timeToGoal(void){
    checkOriginToDestination(getOdometry(), currentGoal_, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
    return timetoCurrentGoal_; //protected variable so can be accessed from other functions in classes (base and derived)
}

double Controller::distanceTravelled(void){//Returns total distance travelled up to the current point in time (updates when called)
    return distanceTravelled_;
}

double Controller::timeTravelled(void){//Calculates total time in motion up to the current point in time (updates when called)
    return timeTravelled_;
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