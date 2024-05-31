// These headers are essential for the test
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// These headers are needed for the code being tested as we use laser data
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
// You will need to add headers for any other messages that you use

// Include the header file for the class that you are testing
// relative to this file
#include "../src/laserprocessing.h"

// We state the test suite name and the test name
TEST(AdvGoalConesTest,GoalWithinCones){
    // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    // We read the bag file to get the messages
    // The bag is in the data folder of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_skeleton");//If you chnage package name you need to change this
    std::string bag_filename=package_share_directory + "/data/goalWithinCones"; // The data for the bag are in the data folder and each folder (such as position1) has a bag file

    rosbag2_cpp::Reader reader; // Create a reader object

    // For each message type we need a serialization object specifci ti data type
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializationLaser;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serializationOdo;
    // We need a message object for laserscan
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    // We also need a message object for odometry
    nav_msgs::msg::Odometry::SharedPtr odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    // Open the bag file
    reader.open(bag_filename);
    // We will loop the messages until we find one of the topic we are interested in and then we will process it
    // Below we break the loop as soon as we have the one message
    // If you need multiple messages you need to adjust this loop to break when your have both messages
    int count = 0;
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name == "/orange/laserscan") {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            serializationLaser.deserialize_message(&serialized_msg, laser_msg.get());
            count++;
        }
        else if (msg->topic_name == "/orange/odom") {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            serializationOdo.deserialize_message(&serialized_msg, odo_msg.get());
            count++;
        }

        if(count == 2){break;}
    }

    ////////////////////////////////////////////
    // Our code is tested below

    // We create an object of the class we are testing
    LaserProcessing laserProcessing(*laser_msg); // We pass the message to the constructor
    std::vector<geometry_msgs::msg::Point> cones = laserProcessing.detectConeCentres();
    EXPECT_EQ(cones.size(), 5); //ground truth from visual inspection of the rvis of the rosbag

    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> pairs = laserProcessing.detectRoad(cones);
    geometry_msgs::msg::Pose pose = odo_msg->pose.pose;
    geometry_msgs::msg::Point transformedPoint1, transformedPoint2;
    //calculate transform of car in global frame
    tf2::Vector3 audiPoint(pose.position.x, pose.position.y, pose.position.z);
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Transform transform(q,audiPoint);
    geometry_msgs::msg::Point point;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> transformedPairs;

    //Apply car transform to cones in road pairs
    for(size_t i=0; i<pairs.size(); i++){
        point = pairs.at(i).first; //transform of first in pair
        tf2::Vector3 conePoint1(point.x+3.75, point.y, point.z);
        tf2::Vector3 transformedVecPoint = transform * conePoint1;
        transformedPoint1.x = transformedVecPoint.x();
        transformedPoint1.y = transformedVecPoint.y();
        transformedPoint1.z = transformedVecPoint.z();
        point = pairs.at(i).second; //transform of second in pair
        tf2::Vector3 conePoint2(point.x+3.75, point.y, point.z);
        transformedVecPoint = transform * conePoint2;
        transformedPoint2.x = transformedVecPoint.x();
        transformedPoint2.y = transformedVecPoint.y();
        transformedPoint2.z = transformedVecPoint.z();

        transformedPairs.push_back(std::make_pair(transformedPoint1, transformedPoint2)); //global cone pairs
    }
    std::pair<double, double> euDist;
    float tolerance = 3;
    double estimatedRoadDist = 4;
    bool goalReachable = false;
    geometry_msgs::msg::Point goal;
    int pairPos;
    //First goal point
    for (size_t i=0;i<pairs.size(); i++){//iterate through road pairs
        //calculate road centre as goal (midpoint of pair)
        goal.x = (transformedPairs.at(i).first.x + transformedPairs.at(i).second.x)/2;
        goal.y = (transformedPairs.at(i).first.y + transformedPairs.at(i).second.y)/2;
    }
    for (size_t j=0; j<transformedPairs.size(); j++){
        euDist.first = std::hypot(transformedPairs.at(j).first.x - goal.x,transformedPairs.at(j).first.y - goal.y);
        euDist.second = std::hypot(transformedPairs.at(j).second.x - goal.x,transformedPairs.at(j).second.y - goal.y);
        // std::cout<<euDist.first<<" "<<euDist.second;
        if(std::abs(estimatedRoadDist - euDist.first) < tolerance && std::abs(estimatedRoadDist - euDist.second) < tolerance){
            goalReachable = true; //sets goal as reachable
            pairPos = j; //record position of road pair for testing
            break;
        }
    }
    ASSERT_TRUE(goalReachable);
    //check road pairs are the first cones in the simulation
    EXPECT_NEAR(transformedPairs.at(pairPos).second.x, 38.313, 0.5);
    EXPECT_NEAR(transformedPairs.at(pairPos).second.y, 17.302, 0.5);
    EXPECT_NEAR(transformedPairs.at(pairPos).first.x, 38.371, 0.5);
    EXPECT_NEAR(transformedPairs.at(pairPos).first.y, 9.348, 0.5);
    //check goal point is roughly in the middle of the simulation's first cone pair
    EXPECT_NEAR(goal.x, 37.969, 0.5);
    EXPECT_NEAR(goal.y, 13.269, 0.5);
}