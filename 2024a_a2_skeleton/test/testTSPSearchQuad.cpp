#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"
#include "test_helper.h"

//Student defined libraries
#include "quadcopter.h"
#include "mission.h"
#include "tsp.h"

///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(TSP2, TSPSearchQuadcopter) {

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter()); //create object of Ackerman class

    //create goal points to search for Quadcopter
    std::vector<pfms::geometry_msgs::Point> goalsQuad;   
    goalsQuad.push_back({ 7, -9});
    goalsQuad.push_back({ -6, -6});
    goalsQuad.push_back({ -5, 9});
    goalsQuad.push_back({ 1, -8});
    goalsQuad.push_back({ 5, -4}); 

    //expected order
    std::vector<int> expectedOrder = {4,0,3,1,2};

    //create mission object
    Mission mission(controllers);
    //initialise variables
    std::vector<double> distancesFromOrigin_;
    std::vector<tsp::Point> goalPoints(goalsQuad.size(), {0,0,0});
    double time;
    pfms::nav_msgs::Odometry origin, estimatedGoalPose;
    origin = populateOdoUGV(0,-2,0);

    distancesFromOrigin_.resize(goalsQuad.size()); //Resize private data member vector of distances from the current platform origin to fit the number of goals
    //Start TSP
    for (int j=0; j<goalsQuad.size(); j++){ //iterate through goals
        //calculate distance from current platform odometry to the current goal (to be used in identifying shortest path)
        controllers.at(0)->checkOriginToDestination(origin, goalsQuad.at(j), distancesFromOrigin_.at(j), time, estimatedGoalPose);
    }
    for (int i=0; i<goalsQuad.size(); i++){ //iterate through goals
        //Copy goal points from missionGoals into the tsp Point struct
        goalPoints.at(i).x = goalsQuad.at(i).x;
        goalPoints.at(i).y = goalsQuad.at(i).y;
        goalPoints.at(i).z = goalsQuad.at(i).z;
    }

    TSP tsp;

    AdjacencyList graph = tsp.generateGraph(goalPoints);

    std::vector<int> actualOrder = tsp.bestPathSearch(graph,distancesFromOrigin_);

    ASSERT_EQ(actualOrder.size(), expectedOrder.size()); //check size of vector is correct
    ASSERT_TRUE(actualOrder == expectedOrder);//Checking order is correct

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}