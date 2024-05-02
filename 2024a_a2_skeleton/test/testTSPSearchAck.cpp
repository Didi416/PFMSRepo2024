#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"
#include "test_helper.h"

//Student defined libraries
#include "ackerman.h"
#include "mission.h"
#include "tsp.h"

///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(TSP, TSPSearchAckerman) {

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman()); //create object of Ackerman class

    //create goal points to search for Ackerman
    std::vector<pfms::geometry_msgs::Point> goalsAck;
    goalsAck.push_back({ 10, -20});    
    goalsAck.push_back({ 30, -15});
    goalsAck.push_back({ 20, 30});
    goalsAck.push_back({ 10, 10});
    goalsAck.push_back({ 30, 10});

    //expected order
    std::vector<int> expectedOrder = {3,2,4,1,0};

    //create mission object
    Mission mission(controllers);
    //initialise variables
    std::vector<double> distancesFromOrigin_;
    std::vector<tsp::Point> goalPoints(goalsAck.size(), {0,0,0});
    double time;
    pfms::nav_msgs::Odometry origin, estimatedGoalPose;
    origin = populateOdoUGV(0,2,0); //starting point

    distancesFromOrigin_.resize(goalsAck.size()); //Resize private data member vector of distances from the current platform origin to fit the number of goals
    //Start TSP
    for (int j=0; j<goalsAck.size(); j++){ //iterate through goals
        //calculate distance from current platform odometry to the current goal (to be used in identifying shortest path)
        controllers.at(0)->checkOriginToDestination(origin, goalsAck.at(j), distancesFromOrigin_.at(j), time, estimatedGoalPose);
    }
    for (int i=0; i<goalsAck.size(); i++){ //iterate through goals
        //Copy goal points from missionGoals into the tsp Point struct
        goalPoints.at(i).x = goalsAck.at(i).x;
        goalPoints.at(i).y = goalsAck.at(i).y;
        goalPoints.at(i).z = goalsAck.at(i).z;
    }

    TSP tsp;

    AdjacencyList graph = tsp.generateGraph(goalPoints); //generate graph from goal points

    std::vector<int> actualOrder = tsp.bestPathSearch(graph,distancesFromOrigin_); //search graph for shortest path

    ASSERT_EQ(actualOrder.size(), expectedOrder.size()); //check size of order vector is correct
    ASSERT_TRUE(actualOrder == expectedOrder);//Checking order is correct

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}