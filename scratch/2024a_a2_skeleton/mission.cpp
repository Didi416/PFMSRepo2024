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
            std::cout<<"BASIC MODE"<<std::endl;
            controllers_.at(a)->setGoals(missionGoals_);
            break;
        case mission::Objective::ADVANCED:
            std::cout<<"ADVANCED TSP MODE"<<std::endl;
            graph = generateGraph(a);
            order = bestPathSearch(graph);
            for (int i=0; i<order.size(); i++){
                tempGoals.push_back(missionGoals_.at(order.at(i)));
            }
            missionGoals_.assign(tempGoals.begin(), tempGoals.end());
            controllers_.at(a)->setGoals(missionGoals_);
            break;
        case mission::Objective::SUPER:
            std::cout<<"SUPER MODE"<<std::endl;
            controllers_.at(a)->setGoals(missionGoals_);
            break;
    }

    for(int i=0; i<missionGoals_.size(); i++){
        platGoalAssoc_.push_back(std::make_pair(a,order.at(i)));
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

std::vector<int> Mission::bestPathSearch(AdjacencyList graph){
    //start tsp search, focusing on movng to the goal with the next shortest distance from current goal
    std::vector<int> order;
    //Let's list the nodes in a vector, and we can do permutations of
    //it as all of the nodes are connected
    std::vector<int> nodes;
    for (unsigned int i=0;i< graph.size();i++){
        nodes.push_back(i);
    }

    //We save the total path as a very large value (default)
    double minDistance = 1e6;

    //This while loop creates all possible permutations of the nodes
    //We can use this while loop to create an order of ID's visited
    //Let's look for te total path to visit nodes in current node order 
    do
    {
        bool OK=true;//We will use this to abolish search if needed
        unsigned int idx=1;//Let's start from index 1 to end 
        double dist=0; // Current distance that we have travelled throug nodes
        while((idx<nodes.size())&&(OK)){
            //We have two nodes 
            unsigned int node1 = nodes.at(idx-1);
            unsigned int node2 = nodes.at(idx);
            //We find in adjacency list node 1 connection to node2 and second element
            //in teh pair is the distance between these nodes
            dist+=graph.at(node1).at(node2).second;
            // std::cout << dist << " "; // This printed distance in debug mode
            //we can abolish search if we are already over the min distance
            if(dist>minDistance){
                OK=false;
            }
            idx++; // Otherwise we increment to next node
        }
        dist += distancesFromOrigin_.at(nodes.at(0));
        std::cout<<"Current Min Distance: "<<minDistance<<" and current distance: "<<dist<<"For: "<<nodes.at(0)<<nodes.at(1)<<nodes.at(2)<<nodes.at(3)<<nodes.at(4)<<std::endl;
        if(dist<minDistance){
            minDistance=dist; // Save minimum distance
            order.clear(); // clear the current order of nodes
            order=nodes; // Save the order of nodes
        }
    } 
    while (std::next_permutation(nodes.begin(), nodes.end()));

    return order;
}

AdjacencyList Mission::generateGraph(int controller){
    AdjacencyList graph (missionGoals_.size());
    distancesFromOrigin_.resize(missionGoals_.size());
    double dist, time;
    pfms::nav_msgs::Odometry origin, estimatedGoalPose;
    origin = controllers_.at(controller)->getOdometry();
    for (int j=0; j<missionGoals_.size(); j++){
        if(controllers_.at(controller)->checkOriginToDestination(origin, missionGoals_.at(j), distancesFromOrigin_.at(j), time, estimatedGoalPose)){
            std::cout<<"Platform can reach goal "<<j<<" from start pos in "<<distancesFromOrigin_.at(j)<<" metres."<<std::endl;
        }
    }
    for (int i=0; i<missionGoals_.size(); i++){
        origin.position.x = missionGoals_.at(i).x;
        origin.position.y = missionGoals_.at(i).y;
        origin.position.z = missionGoals_.at(i).z;
        origin.yaw = origin.yaw;
        origin.linear.x = 0;
        origin.linear.y = 0;
        for (int j=0; j<missionGoals_.size(); j++){
            if(controllers_.at(controller)->checkOriginToDestination(origin, missionGoals_.at(j), dist, time, estimatedGoalPose)){
                dist = sqrt(pow(origin.position.x - missionGoals_.at(j).x, 2) + pow(origin.position.y - missionGoals_.at(j).y, 2));
                graph.at(i).push_back(std::make_pair(j,dist));
                std::cout<<"Platform can reach goal "<<j<<" from goal "<<i<<" in "<<dist<<" metres."<<std::endl;
            }
            else{
                graph.at(i).push_back(std::make_pair(j,0));
                std::cout<<"Platform cannot reach goal "<<j<<" from goal "<<i<<" in "<<dist<<" metres."<<std::endl;
            }
        }
    }
    return graph;
}