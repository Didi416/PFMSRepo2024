#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Mission::Mission(std::vector<ControllerInterface*> controllers){
    controllers_ = controllers;
}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    missionGoals_ = goals;
    for(int i=0; i<missionGoals_.size(); i++){
        for (int j=0; j<controllers_.size(); j++){
            if (controllers_.at(j)->setGoal(missionGoals_.at(i))){
                platGoalAssoc_.push_back(j);
                break;
            }
        }
        if (platGoalAssoc_.size() < missionGoals_.size()){
            // std::cout<<"Platforms cannot reach goal "<<i<<std::endl;
        }
    }
}

void Mission::setMissionObjective(mission::Objective objective){
    missionObjective_ = objective;
}

std::vector<unsigned int> Mission::getPlatformGoalAssociation(){
    return platGoalAssoc_;
}

bool Mission::runMission(){
    for (int i=0; i<missionGoals_.size(); i++){
        controllers_.at(platGoalAssoc_.at(i))->setGoal(missionGoals_.at(i));
        if (!controllers_.at(platGoalAssoc_.at(i))->reachGoal()){
            return false;
        }
    }
    return true;
}

std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> distanceTravelled;
    for (int i=0; i<controllers_.size(); i++){
        distanceTravelled.push_back(controllers_.at(i)->distanceTravelled());
    }

    return distanceTravelled;
}

std::vector<double> Mission::getTimeMoving(){
    std::vector<double> timeMoving;
    for (int i=0; i<controllers_.size(); i++){
        timeMoving.push_back(controllers_.at(i)->timeInMotion());
    }
    return timeMoving;
}