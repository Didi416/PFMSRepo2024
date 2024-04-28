#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include "controller.h"

Mission::Mission(std::vector<ControllerInterface*> controllers){
    controllers_ = controllers;
    totalMissionDistance_.resize(controllers.size());
    totalMissionTime_.resize(controllers.size());
    status_.resize(controllers.size());
}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform){
    missionGoals_ = goals;
    int a = 0;
    for (int i=0; i<controllers_.size(); i++){
        if (controllers_.at(i)->getPlatformType() == platform){
            a = i;
            break;
        }
    }

    controllers_.at(a)->setGoals(missionGoals_);
    pfms::nav_msgs::Odometry origin, estimatedGoalPose;
    origin = controllers_.at(a)->getOdometry();
    double dist, time;
    for(int i=0; i<missionGoals_.size(); i++){
        platGoalAssoc_.push_back(std::make_pair(i,a));
        controllers_.at(a)->checkOriginToDestination(origin, missionGoals_.at(i), dist, time, estimatedGoalPose);
        origin = estimatedGoalPose;
        totalMissionDistance_.at(a) += dist;
        std::cout<<"Total Distance: "<<totalMissionDistance_.at(a)<<std::endl;
    }
} 

bool Mission::run(){
    for (auto controller : controllers_){
        controller->run();
    }
    return true;
}

std::vector<unsigned int> Mission::status(void){
    for (int i=0;i<controllers_.size(); i++){
        // std::cout<<"Total Dist: "<<i<<" -> "<<totalMissionDistance_.at(i)<<" and Distance Trav: "<<getDistanceTravelled().at(i)<<std::endl;
        double percentage = (getDistanceTravelled().at(i)/totalMissionDistance_.at(i))*100;
        if(controllers_.at(i)->status() == pfms::PlatformStatus::IDLE){
            percentage = 100;
        }
        if(percentage >= 100 && controllers_.at(i)->status() != pfms::PlatformStatus::IDLE){
            percentage = 99;
        }
        status_.at(i) = percentage;
    }
    return status_;
}

void Mission::setMissionObjective(mission::Objective objective){
    objective = objective_;
}

std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> distanceTravelled;
    for (int i=0;i<controllers_.size(); i++){
        distanceTravelled.push_back(controllers_.at(i)->distanceTravelled());
    }
    return distanceTravelled;
}

std::vector<double> Mission::getTimeMoving(){
    std::vector<double> timeMoving;
    for (int i=0;i<controllers_.size(); i++){
        timeMoving.push_back(controllers_.at(i)->timeTravelled());
    }
    return timeMoving;
}

std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation(){
    return platGoalAssoc_;
}
