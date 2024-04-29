#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "mission.h"

#include "pfms_types.h"
#include <cmath>
// Some helper header for assembling messages and testing
#include "a1_test_helper.h"
#include "ackerman_test.h"
#include "pfmshog.h"
using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(AckermanExTest, ReachGoals) {

    //We set the initial position of the vehicle
    Odometry odo = populateOdo(0,2,0);
    
    //We create a PfmsHog object which we use to "teleport" the vehicle to the initial position
    pfms::PlatformType platform = pfms::PlatformType::ACKERMAN;
    std::shared_ptr<PfmsHog> pfmsHogPtr = std::make_shared<PfmsHog>(platform);
    pfmsHogPtr->teleport(odo);

    //We create a vector of controllers and add the Ackerman controller to it
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=10,y=0,z=0;
    pfms::geometry_msgs::Point pt1{10,0,0};
    //We set a tolerance for the goal reaching
    controllers.front()->setTolerance(0.5);

    //We set the goal for the Ackerman controller, check it's reachable and get the distance and time to reach the goal
    bool reachable = controllers.at(0)->setGoal(pt1);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;
    

    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogPtr->setGoal(pt1);

    //This now triggers the Ackerman to reach the goal and "blocks" until the goal is reached
    bool reached = controllers.at(0)->reachGoal();

    //We now check that the goal has been reached (the reachGoal function returns when te goal is reached)
    ASSERT_TRUE(reached);

    //We now check that the goal has been reached using the PfmsHog
    std::vector<double> distances;
    bool reachedCheck = pfmsHogPtr->checkGoalsReached(distances);
    std::cout << "Goal reach check:" << reachedCheck << std::endl;
    ASSERT_TRUE(reachedCheck);

    //We can also check the diatance to the goal reported by PfmsHog
    std::cout << "Goal size:" << distances.size() << std::endl;
    ASSERT_EQ(distances.size(),1);
    std::cout << "Distance to goal:" << distances.at(0) << std::endl;
    ASSERT_NEAR(distances.at(0),0,1.0);


    //Let's repeat the process with a new goal
    pfms::geometry_msgs::Point pt2{9,8};

    reachable = controllers.at(0)->setGoal(pt2);
    ASSERT_TRUE(reachable);

    dist = controllers.at(0)->distanceToGoal();
    t = controllers.at(0)->timeToGoal();
    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;


    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogPtr->setGoal(pt2);


    //This now triggers the Ackerman to reach the goal and "blocks" until the goal is reached
    reached = controllers.at(0)->reachGoal();

    //We now check that the goal has been reached (the reachGoal function returns when te goal is reached)
    ASSERT_TRUE(reached);

    //We now check that the goal has been reached using the PfmsHog
    reachedCheck = pfmsHogPtr->checkGoalsReached(distances);
    ASSERT_TRUE(reachedCheck);

    //We can also check the diatance to the goal reported by PfmsHog
    ASSERT_EQ(distances.size(),1);
    std::cout << "Distance to goal:" << distances.at(0) << std::endl;
    ASSERT_NEAR(distances.at(0),0,1.0);

}

TEST(AckermanExTest, ThreeGoals) {

    //! @todo
    //! Create another test with two goals 

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
