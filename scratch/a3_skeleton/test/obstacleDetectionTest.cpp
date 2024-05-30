// These headers are essential for the test
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// These headers are needed for the code being tested as we use laser data
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
// You will need to add headers for any other messages that you use

// Include the header file for the class that you are testing
// relative to this file
#include "../src/laserprocessing.h"

// We state the test suite name and the test name
TEST(ObstacleDetect,FiretruckPresent){

    // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    // We read the bag file to get the messages
    // The bag is in the data folder of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_skeleton");//If you chnage package name you need to change this
    std::string bag_filename=package_share_directory + "/data/firetruckDetection"; // The data for the bag are in the data folder

    rosbag2_cpp::Reader reader; // Create a reader object

    // For each message type we need a serialization object specifci ti data type
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    // We also need a message object (shared pointer in this case)
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    // Open the bag file
    reader.open(bag_filename);
    // We will loop the messages until we find one of the topic we are interested in and then we will process it
    // Below we break the loop as soon as we locate the laserscan topic
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name != "/orange/laserscan") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        break;
    }

    ////////////////////////////////////////////
    // Our code is tested below

    // We create an object of the class we are testing
    LaserProcessing laserProcessing(*laser_msg); // We pass the message to the constructor

    // We call the function that we are testing
    bool detectFiretruck = laserProcessing.detectLargeObstacle();

    // We check if the function returned the expected value, in this case returns true for locating a firetruck within 10units of the car.
    ASSERT_TRUE(detectFiretruck);
}

TEST(ObstacleDetect,FiretruckNotPresent){

    // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    // We read the bag file to get the messages
    // The bag is in the data folder of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_skeleton");//If you chnage package name you need to change this
    std::string bag_filename=package_share_directory + "/data/firetruckDetectionFalse"; // The data for the bag are in the data folder

    rosbag2_cpp::Reader reader; // Create a reader object

    // For each message type we need a serialization object specifci ti data type
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    // We also need a message object (shared pointer in this case)
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    // Open the bag file
    reader.open(bag_filename);
    // We will loop the messages until we find one of the topic we are interested in and then we will process it
    // Below we break the loop as soon as we locate the laserscan topic
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name != "/orange/laserscan") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        break;
    }

    ////////////////////////////////////////////
    // Our code is tested below

    // We create an object of the class we are testing
    LaserProcessing laserProcessing(*laser_msg); // We pass the message to the constructor

    // We call the function that we are testing
    bool detectFiretruck = laserProcessing.detectLargeObstacle();

    // We check if the function returned the expected value, in this case returns false as firetruck is outside limit of 10 units from the car.
    //Ground Truth determined by manually placing the firetruck
    ASSERT_FALSE(detectFiretruck);
}

// TEST(ObstacleDetect,FiretruckDriving){
//     // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

//     // We read the bag file to get the messages
//     // The bag is in the data folder of the package
//     std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_skeleton");
//     std::string bag_filename = package_share_directory + "/data/firetruckDetectionDriving";

//     rosbag2_cpp::Reader reader;
//     reader.open(bag_filename);

//     // Define the topics of interest
//     std::vector<std::string> topics_of_interest = {"/orange/laserscan", "/orange/brake_cmd", "/orange/throttle_cmd", "/orange/odom"};

//     // Create serialization objects for each topic (in order of topics of interest)
//     std::vector<rclcpp::SerializationBase> serializations;
//     serializations.push_back(rclcpp::Serialization<sensor_msgs::msg::LaserScan>());
//     serializations.push_back(rclcpp::Serialization<std_msgs::msg::Float64>());
//     serializations.push_back(rclcpp::Serialization<std_msgs::msg::Float64>());
//     serializations.push_back(rclcpp::Serialization<nav_msgs::msg::Odometry>());

//     // Create message objects for each topic
//     sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
//     std_msgs::msg::Float64::SharedPtr brake_msg = std::make_shared<std_msgs::msg::Float64>();
//     std_msgs::msg::Float64::SharedPtr throttle_msg = std::make_shared<std_msgs::msg::Float64>();
//     nav_msgs::msg::Odometry::SharedPtr odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

//     while (reader.has_next()) {
//         rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

//         // Check if the topic is of interest
//         // if (std::find(topics_of_interest.begin(), topics_of_interest.end(), msg->topic_name) != topics_of_interest.end()) { //if topic name is in the list of interested topics
//         //     rclcpp::SerializedMessage serialized_msg(*msg->serialized_data); 
            
//         //     serialization->deserialize_message(&serialized_msg, message.get());

//         //     // Process the message
//         //     std::cout << "Received message on topic " << msg->topic_name << std::endl;
//         //     // ...
//         // }
//     }

//     ////////////////////////////////////////////
//     // Our code is tested below

//     // We create an object of the class we are testing
//     LaserProcessing laserProcessing(*laser_msg); // We pass the message to the constructor

//     // We call the function that we are testing
//     bool detectFiretruck = laserProcessing.detectLargeObstacle();

//     // We check if the function returned the expected value, in this case returns true for locating a firetruck within 10units of the car.
//     ASSERT_TRUE(detectFiretruck);
// }
