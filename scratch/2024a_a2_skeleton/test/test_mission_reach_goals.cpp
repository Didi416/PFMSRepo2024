#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "ackerman.h"
#include "quadcopter.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "pfmshog.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(MissionReachGoals, ThreeGoalsEach) {

    // We create two PfmsHog objects, one for each platform
    // We teleport the platforms to starting locations
    std::unique_ptr<PfmsHog> pfmsHogAudiPtr = std::make_unique<PfmsHog>(pfms::PlatformType::ACKERMAN);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        pfmsHogAudiPtr->teleport(odo);
    }
    std::unique_ptr<PfmsHog> pfmsHogQuadPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    {
        Odometry odo = populateOdoUAV(0,-2,2,0);
        pfmsHogQuadPtr->teleport(odo);
    }

    // We create two controllers, one for each platform
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.5);
    controllers.back()->setTolerance(0.5);

    // The goals for the Ackerman platform
    std::vector<pfms::geometry_msgs::Point> goalsAck;
    goalsAck.push_back({ 20, 0});
    goalsAck.push_back({ 30, -10});
    goalsAck.push_back({ 20, -20});
    goalsAck.push_back({ -20, -20});
    goalsAck.push_back({ -30, -10});
    goalsAck.push_back({ -20, 0});
    goalsAck.push_back({ 0, 2});
    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogAudiPtr->setGoals(goalsAck);

    // The goals for the Quadcopter platform
    std::vector<pfms::geometry_msgs::Point> goalsQuad;
    goalsQuad.push_back({ 10, -10});
    goalsQuad.push_back({ 0, -20});
    goalsQuad.push_back({ -10, -10});
    goalsQuad.push_back({ 0, -2});
    pfmsHogQuadPtr->setGoals(goalsQuad);

    // We now have controller and goals, let's set up mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::BASIC);
    mission.setGoals(goalsAck,pfms::PlatformType::ACKERMAN);
    std::cout<<"Set Ackerman Goals"<<std::endl;
    mission.setGoals(goalsQuad,pfms::PlatformType::QUADCOPTER);
    std::cout<<"Set Quad Goals"<<std::endl;

    auto start_call_mission_time = std::chrono::system_clock::now();
    // We run mission  (non blocking)
    mission.run();
    std::cout<<"Run Mission"<<std::endl;
    auto current_time = std::chrono::system_clock::now();
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_call_mission_time);

    EXPECT_NEAR(time_taken.count(),0,2.0);//This checks if run mission is a blocking call, should return quickly
    std::cout<<"Time Taken: "<<time_taken.count()<<std::endl;

    // We have an estimated time to reach goal, which is conservative usually vehicles reaches it in
    // less time. 
    //
    // Just in case students have not implemented this correctly, we have an upper bound of 90s
    // We will loop until that time and if the goal is not reached until then (or we have status
    // indicating IDLE, we know it has been reached
    auto start_time = std::chrono::system_clock::now();
    double maxTime =180.0; // We impose a maximum time of 180s

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
        std::cout<<"Check Status"<<std::endl;
        //Let's check the status
        std::vector<unsigned int> status = mission.status();
        //Status shoud have two elements
        std::cout<<"Status size: "<<status.size()<<std::endl;
        ASSERT_EQ(status.size(),2);

        // std::cout << "progress ..." << status.front() << "% " <<  status.back() << "% " << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // The status should be less than 100% for both platforms
        // Thus indicating still completing mission
        bool completed = true;
        for (auto st:status){
            if(st < 100){
                //mission accomplished
                completed=false;
            }
        }

        // If both platforms have completed the mission, we are done
        if(completed){
            OK=true;
        }
        else{
            //Let's slow down this loop to 200ms (5Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    ASSERT_FALSE(timeExceeded); // time should not be exceeded

    // We now check that the goal has been reached using the PfmsHog
    std::vector<double> distances;
    {
        bool reachedCheck = pfmsHogAudiPtr->checkGoalsReached(distances);
        ASSERT_TRUE(reachedCheck);
    }
    {
        bool reachedCheck = pfmsHogQuadPtr->checkGoalsReached(distances);
        ASSERT_TRUE(reachedCheck);
    }

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
