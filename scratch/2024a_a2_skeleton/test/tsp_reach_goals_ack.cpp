#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "ackerman.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(TSPQuadcopter, ThreeGoals) {

    //We create the PfmHog object pointer and use it to set initial pose of Ackerman for test   
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdo(0,2,0);
        pfmsHogPtr->teleport(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

    std::vector<pfms::geometry_msgs::Point> goalsSupplied;
    goalsSupplied.push_back({ -20, -20});    
    goalsSupplied.push_back({ 20, 0});
    goalsSupplied.push_back({ 20, -20});
    goalsSupplied.push_back({ -30, -10});
    goalsSupplied.push_back({ 30, -10});

    std::vector<int> order = {3,0,2,4,1};

    std::vector<pfms::geometry_msgs::Point> goalsOrder;
    goalsOrder.push_back(goalsSupplied.at(order.at(0)));
    goalsOrder.push_back(goalsSupplied.at(order.at(1)));
    goalsOrder.push_back(goalsSupplied.at(order.at(2)));
    goalsOrder.push_back(goalsSupplied.at(order.at(3)));
    goalsOrder.push_back(goalsSupplied.at(order.at(4)));

    pfmsHogPtr->setGoals(goalsOrder);

    // We now have controller and goals, let's set up mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::ADVANCED);
    mission.setGoals(goalsSupplied,pfms::PlatformType::ACKERMAN);

    bool reachable = mission.run();

    ASSERT_TRUE(reachable);

    if(reachable){
        std::cout << "All goals reachable" << std::endl;
    }
    else{
        std::cout << "Goals NOT reachable" << std::endl;
    }


    std::vector<std::pair<int, int>>  association = mission.getPlatformGoalAssociation();
    std::vector<int> computedOrder;

    for(unsigned int i=0;i<association.size();i++){
        std::cout << i << " " << association.at(i).first << " -- " << association.at(i).second << std::endl;
        computedOrder.push_back(association.at(i).second);
    }
    

    ASSERT_TRUE(order == computedOrder);//Checking order is correct

    // We have an estimated time to reach goal, which is conservative usually vehicles reaches it in
    // less time. Nevertheless we could use 3x this time as the max time to reach goal (factor of safety)
    //
    // We will loop until that time and if the goal is not reached until then (or we have status
    // indicating IDLE, we know it has been reached

    auto start_time = std::chrono::system_clock::now();
    double maxTime =180.0;
    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded

    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        //std::chrono::seconds is integer for some reason, thus duration<double>
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(maxTime)){
            //We have now taken time
            timeExceeded=true;
            OK=true;
        }

        std::vector<unsigned int> status = mission.status();

        if (status.front() >= 100 && status.back() >= 100)
        { // we check progess, at 100 mission is acomplished
            // mission accomplished
            OK = true;
        }
        else
        {
            // std::cout << "stat: "<<controllers.front()->status() << " ";
            // std::cout << "progress "<< status.front();
        }

        //Let's slow down this loop to 200ms (5Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // std::cout << "\r";
    }

    EXPECT_FALSE(timeExceeded); // time shoudl not be exceeded

    std::vector<double> distances;
    bool reached = pfmsHogPtr->checkGoalsReached(distances);
    ASSERT_TRUE(reached); // All should be reached
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
