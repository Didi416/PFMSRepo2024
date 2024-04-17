#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Mission::Mission(std::vector<ControllerInterface*> controllers){
    controllers_ = controllers;
    totalDistance_ = 0;
    totalTime_ = 0;
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
    for(int i=0; i<missionGoals_.size(); i++){
        // platGoalAssoc_.push_back(std::make_pair(i,a));
        totalDistance_ += controllers_.at(a)->distanceToGoal();
        // totalTime_ += controllers_.at(i)->timeToGoal();
        // std::cout<<"Total Distance: "<<totalDistance_<<" and Total Time: "<<totalTime_<<std::endl;
        std::cout<<"Allocated Goal: "<<i<<" of "<<missionGoals_.size()<<" to "<<a<<std::endl;
    }
}

bool Mission::run(){
    for (auto controller : controllers_){
        controller->run();
    }
    return true;
}

std::vector<unsigned int> Mission::status(void){
    std::cout<<"Read Status, controller size: "<<controllers_.size()<<std::endl;
    for (int i=0;i<controllers_.size(); i++){
        std::cout<<"Total Dist: "<<totalDistance_<<"Distance Trav: "<<controllers_.at(i)->distanceTravelled()<<std::endl;
        status_.at(i) = int(controllers_.at(i)->distanceTravelled()/totalDistance_);
        std::cout<<"Status Value: "<<status_.at(i)<<std::endl;
    }
    return status_;
}

void Mission::setMissionObjective(mission::Objective objective){
    objective = objective_;
}

std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> distanceTravelled;
    for (int i=0;i<controllers_.size(); i++){
        distanceTravelled.at(i) = controllers_.at(i)->distanceTravelled()/totalDistance_;
    }
    return distanceTravelled;
}

std::vector<double> Mission::getTimeMoving(){
    std::vector<double> timeMoving;
    for (int i=0;i<controllers_.size(); i++){
        timeMoving.at(i) = controllers_.at(i)->distanceTravelled()/totalDistance_;
    }
    return timeMoving;
}

std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation(){
    return platGoalAssoc_;
}
