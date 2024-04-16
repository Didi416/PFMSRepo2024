#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Mission::Mission(std::vector<ControllerInterface*> controllers){

}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform){

}

bool Mission::run(){
    return true;
}

std::vector<unsigned int> Mission::status(void){
    std::vector<unsigned int> status;
    return status;
}

void Mission::setMissionObjective(mission::Objective objective){
    
}

std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> distanceTravelled;
    return distanceTravelled;
}

std::vector<double> Mission::getTimeMoving(){
    std::vector<double> timeMoving;
    return timeMoving;
}

std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation(){
    return platGoalAssoc_;
}
