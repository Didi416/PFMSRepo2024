#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Mission::Mission(std::vector<ControllerInterface*> controllers){

}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals){

}

bool Mission::runMission(){

    return true;
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

std::vector<unsigned int> Mission::getPlatformGoalAssociation(){
    std::vector<unsigned int> platGoalAssoc;

    return platGoalAssoc;
}