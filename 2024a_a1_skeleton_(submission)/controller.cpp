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
    currentOdo_ = getOdometry();
    std::cout<<"Current Odo Readings: "<<std::endl;
    std::cout<<currentOdo_.position.x<<std::endl;
    std::cout<<currentOdo_.position.y<<std::endl;
    std::cout<<currentOdo_.yaw<<std::endl;
    if (checkOriginToDestination(currentOdo_, goal_, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_)){
        return true;
    }
    else{
        return false;
    }
}

pfms::PlatformType Controller::getPlatformType(void){
    return platformType_;//accesses protected value in Controller class storing platform type (Ackerman or SkidSteer)
}

double Controller::distanceToGoal(void){   
    //automatic computing of distance from current position to goal through checkOriginToDestination function in Audi library
    checkOriginToDestination(getOdometry(), goal_, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
    return distanceToCurrentGoal_; //protected variable so can be accessed from other functions in classes( base and derived)
}

double Controller::timeToGoal(void){
    //automatic computing of distance from current position to goal through checkOriginToDestination function in Audi library
    checkOriginToDestination(getOdometry(), goal_, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
    return timetoCurrentGoal_; //protected variable so can be accessed from other functions in classes( base and derived)
}

double Controller::distanceTravelled(void){//Calculates total distance travelled up to the current point in time (updates when called)
    totalDistance_ += (originalDistanceToCurrentGoal_ - distanceToGoal());
    return totalDistance_;
}

double Controller::timeInMotion(void){//Calculates total time in motion up to the current point in time (updates when called)
    totalTime_ += (originalTimetoCurrentGoal_ - timeToGoal());
    return totalTime_;
}

bool Controller::setTolerance(double tolerance){ //used to set the tolerance around the goal that is accepted as reaching the goal
    goalTolerance_ = tolerance;
    return true;
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){ //uses pfmsConnectorPtr to read odometry of the current platform 
    pfmsConnectorPtr_->read(currentOdo_,platformType_);
    return currentOdo_;
}