#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Mission::Mission(std::vector<ControllerInterface*> controllers){
    controllers_ = controllers;
    totalDistance_.resize(controllers.size());
    totalTime_.resize(controllers.size());
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
    for(int i=0; i<missionGoals_.size(); i++){
        platGoalAssoc_.push_back(std::make_pair(i,a));
        totalDistance_.at(a) += (controllers_.at(a)->distanceToGoal());
        totalTime_.at(a) += controllers_.at(a)->timeToGoal();
        std::cout<<"Total Distance: "<<totalDistance_.at(a)<<" and Total Time: "<<totalTime_.at(a)<<std::endl;
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
    for (int i=0;i<controllers_.size(); i++){
        std::cout<<"Total Dist: "<<i<<" -> "<<totalDistance_.at(i)<<" and Distance Trav: "<<getDistanceTravelled().at(i)<<std::endl;
        status_.at(i) = (getDistanceTravelled().at(i)/totalDistance_.at(i));
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
