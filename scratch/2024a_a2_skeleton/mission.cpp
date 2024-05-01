#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include "controller.h"
#include <queue>
#include <set>

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

    pfms::nav_msgs::Odometry origin, estimatedGoalPose;
    origin = controllers_.at(a)->getOdometry();
    double dist, time;
    std::vector<int> order(missionGoals_.size(),0);
    AdjacencyList graph;
    std::vector<pfms::geometry_msgs::Point> tempGoals;

    switch (objective_){
        case mission::Objective::BASIC:
            controllers_.at(a)->setGoals(missionGoals_);
            break;
        case mission::Objective::ADVANCED:
            graph = generateGraph(a);
            order = bestPathSearch(graph);
            for (int i=0; i<order.size(); i++){
                tempGoals.push_back(missionGoals_.at(order.at(i)));
            }
            missionGoals_.assign(tempGoals.begin(), tempGoals.end());
            controllers_.at(a)->setGoals(missionGoals_);
            break;
        case mission::Objective::SUPER:
            controllers_.at(a)->setGoals(missionGoals_);
            break;
    }

    for(int i=0; i<missionGoals_.size(); i++){
        platGoalAssoc_.push_back(std::make_pair(a,order.at(i)));
        controllers_.at(a)->checkOriginToDestination(origin, missionGoals_.at(i), dist, time, estimatedGoalPose);
        origin = estimatedGoalPose;
        totalMissionDistance_.at(a) += dist;
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
    objective_ = objective;
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

std::vector<int> Mission::bestPathSearch(AdjacencyList graph){ // Search the goals graph to find shortest path
    //start tsp search, focusing on moving to the goal with the next shortest distance from current goal
    std::vector<int> order;
    //List all goals in a vector, to do permutations
    std::vector<int> goals;
    for (unsigned int i=0; i< graph.size(); i++){
        goals.push_back(i);
    }
    double minDistance = 1e6; //Set initial minimum distance to a large value, so first permutation is definitely less than

    //Go through all permutations of the order of goals, comparing total distances to 
    //determine the minimum distance possible and asociated permuation:
    do {
        bool OK = true; //Use this to abort search if distanec is already above minimum distance
        unsigned int i = 1;//Start from index 1 to end 
        double dist = 0; // Current distance travelled through goals
        while((i<goals.size()) && (OK)){
            // Checks distance between two goals in the order dictated by the current permutation
            unsigned int goal1 = goals.at(i-1);
            unsigned int goal2 = goals.at(i);
            //We find in the adjacency list (graph), the goal1 connection to goal2 and access the second element which is the distance between goal1 and goal2
            dist += graph.at(goal1).at(goal2).second;
            //We abort current search if dist is already over the min distance
            if(dist > minDistance){
                OK = false;
            }
            i++; // increment to next goal in permutation
        }
        dist += distancesFromOrigin_.at(goals.at(0));
        if(dist<minDistance){
            minDistance=dist; // Save current dist as new minimum distance
            order.clear(); // Clear the current order of goals
            order=goals; // Save the order of goals as current shortest path
        }
    } 
    while (std::next_permutation(goals.begin(), goals.end())); //Go to next permutation of goals order

    return order;
}

AdjacencyList Mission::generateGraph(int controller){ //Generate graph for searching from mission goals
    AdjacencyList graph (missionGoals_.size()); // Create graph of size of mission goals (number of vectors in the vector = missionGoals.size())
    distancesFromOrigin_.resize(missionGoals_.size()); //Resize private data member vector of distances from the current platform origin to fit the number of goals
    double dist, time; // for calculating distance and time in checkOriginToestination function
    pfms::nav_msgs::Odometry origin, estimatedGoalPose; //stores origin or starting position for calculating checkOriginToDestination function
    origin = controllers_.at(controller)->getOdometry(); //initially set origin to platform's current odometry to get distances from origin to all goals
    for (int j=0; j<missionGoals_.size(); j++){ //iterate through goals
        //calculate distance from current platform odometry to the current goal (to be used in identifying shortest path)
        controllers_.at(controller)->checkOriginToDestination(origin, missionGoals_.at(j), distancesFromOrigin_.at(j), time, estimatedGoalPose);
    }
    for (int i=0; i<missionGoals_.size(); i++){ //iterate through goals
        // store current goal point as the current origin to calculate the distances from the current goal to all other goals
        origin.position.x = missionGoals_.at(i).x;
        origin.position.y = missionGoals_.at(i).y;
        origin.position.z = missionGoals_.at(i).z;
        origin.yaw = origin.yaw;
        origin.linear.x = 0;
        origin.linear.y = 0;
        for (int j=0; j<missionGoals_.size(); j++){ //iterate through goals
            if(controllers_.at(controller)->checkOriginToDestination(origin, missionGoals_.at(j), dist, time, estimatedGoalPose)){ //checks if goal can be reached from another goal position
                dist = sqrt(pow(origin.position.x - missionGoals_.at(j).x, 2) + pow(origin.position.y - missionGoals_.at(j).y, 2)); //calculates straight line distance to goals, as Audi library doesn't always calculate the distance correctly
                graph.at(i).push_back(std::make_pair(j,dist)); //add an entry into the graph, to signify that the goal 'j' can be reached from goal 'i' in a distance calculated by 'dist'
            }
            else{ //if a goal cannot be reached from current goal position (e.g. Ackerman cannot reach the goal it is already at)
                graph.at(i).push_back(std::make_pair(j,0)); //still ad an entry, but make the distance equal to 0
            }
        }
    }
    return graph;
}