#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "quadcopter.h" // The quadcopter
#include "pfms_types.h" //A1 types
#include "test_helper.h" // Helper header that assembled the message
#include "pfmshog.h" // Controlling the simulator



using namespace pfms::nav_msgs;

TEST(QuadcopterTest, Constructor) {

    //We create the PfmHog object pointer and use it to set initial pose of Quadcopter for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    {
        Odometry odo = populateOdo(2,-2,0,M_PI/4);
        pfmsHogPtr->teleport(odo);
    }    

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    double tolerance =0.2;
    controllers.front()->setTolerance(tolerance);

    
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceToGoal(),0);//Should be zero
    EXPECT_FLOAT_EQ(controllers.at(0)->timeToGoal(),0); //Should be zero
    EXPECT_FLOAT_EQ(controllers.at(0)->timeInMotion(),0); //Should be zero
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceTravelled(),0); //Should be zero
    EXPECT_EQ(controllers.at(0)->getPlatformType(),pfms::PlatformType::QUADCOPTER);//Quadcopter type

    //If we call getOdometry we expect it to retrun the location we set as initial odometry
    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    EXPECT_NEAR(odo.position.x,2,tolerance);
    EXPECT_NEAR(odo.position.y,-2,tolerance);
}


TEST(QuadcopterTest, Goals) {

    //We create the PfmHog object pointer and use it to set initial pose of Quadcopter for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    {
        Odometry odo = populateOdo(0,0,0,0);
        pfmsHogPtr->teleport(odo);
    }    


    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    double tolerance =0.2;
    controllers.front()->setTolerance(tolerance);

    // We try to reach fisrt point
    {
        //Goal location
        pfms::geometry_msgs::Point pt{4,1};



        // Set the goal for controller zero (the Quad)
        bool reachable = controllers.at(0)->setGoal(pt);

        pfmsHogPtr->setGoal(pt);

        // Get distance to goal
        double dist = controllers.at(0)->distanceToGoal();
        // Get time to goal
        double t = controllers.at(0)->timeToGoal();

        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;

        ASSERT_TRUE(reachable);// The goal should be reachable
        ASSERT_NEAR(dist,4.123,tolerance);//Distance 4.123[m]
        ASSERT_NEAR(t,10.307,0.5); //Time 3.535 [s], tolerance 0.5 [s] for the answer

        bool reached = controllers.at(0)->reachGoal();

        ASSERT_TRUE(reached);
        std::vector<double> distances;
        bool reachedCheck = pfmsHogPtr->checkGoalsReached(distances);
        std::cout << "Goal reach check:" << reachedCheck << " with distance" << distances.at(0) << std::endl;
        ASSERT_TRUE(reachedCheck);
    }

    //The 2nd goal, relies on fact we reached first, ASSERT will prevent below running
    // If we did not reach first goal.
    {
        pfms::geometry_msgs::Point pt{0,-6.5};

        pfmsHogPtr->setGoal(pt);

        bool reachable = controllers.at(0)->setGoal(pt);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;

        ASSERT_TRUE(reachable);
        ASSERT_NEAR(dist,8.20039,tolerance);
        ASSERT_NEAR(t,20.501,0.5);

        bool reached = controllers.at(0)->reachGoal();

        ASSERT_TRUE(reached);

        std::vector<double> distances;
        bool reachedCheck = pfmsHogPtr->checkGoalsReached(distances);
        std::cout << "Goal reach check:" << reachedCheck << " with distance" << distances.at(0) << std::endl;
        ASSERT_TRUE(reachedCheck);
    }

}

TEST(QuadcopterTest, TotalTime) {

    //We create the PfmHog object pointer and use it to set initial pose of Quadcopter for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    {
        Odometry odo = populateOdo(0,0,0,0);
        pfmsHogPtr->teleport(odo);
    }    

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    double tolerance =0.2;
    controllers.front()->setTolerance(tolerance);

    // Will give slightly larger tolerance for testing
    tolerance = 0.5;        

    //Goal location
    pfms::geometry_msgs::Point pt1{2,1.2};
    pfms::geometry_msgs::Point pt2{0,-2.0};

    double distance1= std::pow ( std::pow(pt1.x-0,2)+ std::pow(pt1.y-0,2),0.5);
    double distance2= std::pow ( std::pow(pt2.x-pt1.x,2)+ std::pow(pt2.y-pt1.y,2),0.5);

    // We try to reach fisrt point
    {
        // Set the goal for controller zero (the Quad)
        bool reachable = controllers.at(0)->setGoal(pt1);

        // Get distance to goal
        double dist = controllers.at(0)->distanceToGoal();
        // Get time to goal
        double t = controllers.at(0)->timeToGoal();

        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;


        ASSERT_TRUE(reachable);// The goal should be reachable
        ASSERT_NEAR(dist,distance1,tolerance);
        ASSERT_NEAR(t,distance1/0.4,1.0);

        bool reached = controllers.at(0)->reachGoal();
        ASSERT_TRUE(reached);
    }

    //The 2nd goal, relies on fact we reached first, ASSERT will prevent below running
    // If we did not reach first goal.
    {

        bool reachable = controllers.at(0)->setGoal(pt2);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;

        ASSERT_TRUE(reachable);
        ASSERT_NEAR(dist,distance2,tolerance);
        ASSERT_NEAR(t,distance2/0.4,1.0);

        bool reached = controllers.at(0)->reachGoal();

        ASSERT_TRUE(reached);

    }

    double anticipatedTime = (distance1+distance2)/0.4;
    double toleranceTime = anticipatedTime*0.1; // 10% error accepted

    ASSERT_NEAR(controllers.at(0)->timeInMotion(),anticipatedTime,toleranceTime);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
