#include "controller.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include <thread>

Controller::Controller(){
    distanceTravelled_ = 0;
    timeTravelled_ = 0;
    distanceToCurrentGoal_; 
    timetoCurrentGoal_;
    goalTolerance_ = 0.5; //default goal tolerance for when unit tests do not set it.
    
    running_ = false;
    threads_.push_back(std::thread(&Controller::reachGoals, this));
    platformStatus_ = pfms::PlatformStatus::RUNNING;
}
Controller::~Controller(){
    running_ = false;
    //join threads
    for(auto & thread : threads_){
        thread.join();
    }
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    goals_ = goals;
    pfms::nav_msgs::Odometry origin = getOdometry();
    for (auto goal:goals_){
        // std::cout<<"Current Odo Readings: "<<std::endl;
        // std::cout<<origin.position.x<<std::endl;
        // std::cout<<origin.position.y<<std::endl;
        // std::cout<<origin.position.z<<std::endl;
        // std::cout<<origin.yaw<<std::endl;
        checkOriginToDestination(origin, goal, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
        origin = estimatedGoalPose_;
    }
    return true;
}

void Controller::run(void){
    running_ = true;
    std::unique_lock<std::mutex> lck(mtxStart_);
    mtxStart_.unlock();
    cvStart_.notify_all();
    pfmsConnectorPtr_->send(platformStatus_);
}

double Controller::distanceToGoal(void){
    return distanceToCurrentGoal_; //protected variable so can be accessed from other functions in classes( base and derived)
}

double Controller::timeToGoal(void){
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

bool Controller::updateDistTime(double velocity){
    getOdometry();
    distanceTravelled_ += sqrt(pow((currentOdo_.position.x - previousOdo_.position.x),2)+pow((currentOdo_.position.y - previousOdo_.position.y),2));
    timeTravelled_ = distanceTravelled_/velocity;
    previousOdo_ = currentOdo_;
    return true;
}