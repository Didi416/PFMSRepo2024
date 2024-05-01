#include "sample.h"
/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser)
 */

using std::cout;
using std::endl;

using std::placeholders::_1;


PfmsSample::PfmsSample()
    : Node("laser_example")
{
  //Subscribing to odometry
  sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/orange/odom", 10, std::bind(&PfmsSample::odom_callback, this, _1));

  //Subscribing to laser
  sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "orange/laserscan", 10, std::bind(&PfmsSample::laser_callback,this,_1));

  //thread_(std::thread([this]() -> void {process();}))
  thread_ = new std::thread(&PfmsSample::process,this);
}

PfmsSample::~PfmsSample()
{
  if (thread_->joinable()) {
    thread_-> join();
  }
}



// A callback for odometry
//void PfmsSample::odom_callback(const nav_msgs::msg::Odometry& msg)
void PfmsSample::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    /**
     * @todo - Ex 1: Obtain a pose (x,y yaw) from nav_msgs/Odometry
     *

     * - On command line type 'ros2 interface show nav_msgs/msg/Odometry'
     * - The position and orientation are in two seperate parts of the message
     * - The orinetation is provided as a quaternion
     * - Which angle to we need?
     * - Ros has a 'tf' library with a helper function to get yaw from the quaternion
     * - http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     * - Consider: Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
     * - Where is time of this message stored
     */


    geometry_msgs::msg::Pose pose = msg->pose.pose;

// We can use 2 different verbosty levels, and swicth then ON/OFF dynamically
// RCLCPP_INFO_STREAM(get_logger(),"x: " << pose.position.x
//            << ",  y: " << pose.position.y
//            << ",  yaw: "<< tf2::getYaw(pose.orientation));
//  RCLCPP_DEBUG_STREAM("x: " << msg->pose.pose.position.x
//                << ",  y: " << msg->pose.pose.position.y
//                << ",  yaw: "<< tf::getYaw(msg->pose.pose.orientation));

    std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
    robotPose_ = pose; // We store a copy of the pose in robotPose_
}



void PfmsSample::laser_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{

  /**
   * @todo - Ex 1 : Find the closest point {x,y} to the robot using sensor_msgs::LaserScan
   *
   * On command line type 'ros2 interface show sensor_msgs/msg/LaserScan'
   * What are we provided in this message?
   * Do we have the information in this message to find the closest point?
   * What part of the message do we need to iterate over?
   * How do we convert from range data to {x,y} [this is known as polar to cartesian](https://www.mathsisfun.com/polar-cartesian-coordinates.html)
   * Where is time of this message stored?
   * Is the closest point identified the same as the one you see as closest on the stage simulator? Why is this the case?
   */

    std::unique_lock<std::mutex> lck(laserDataMtx_);
    laserData_ = *msg; // We store a copy of the LaserScan in laserData_
    lck.unlock();
}

void PfmsSample::process()
{
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
        std::unique_lock<std::mutex> lck2 (laserDataMtx_);
        RCLCPP_INFO_STREAM(get_logger(),"Robot Pose: x: " << robotPose_.position.x
               << ",  y: " << robotPose_.position.y
               << ",  yaw: "<< tf2::getYaw(robotPose_.orientation));

        //If we have not created the laserProcessingPtr object, create it
        if(laserProcessingPtr_ == nullptr){
            laserProcessingPtr_ = std::make_unique<LaserProcessing>(laserData_);
        }
        else{
            laserProcessingPtr_->newScan(laserData_);
        }

        geometry_msgs::msg::Point pt = laserProcessingPtr_->closestPoint();
        RCLCPP_INFO_STREAM(get_logger(),"Closest Point: x: " << pt.x << ", y: " << pt.y);

	//Now to compute the point in global reference frame, use the coe you developed in Quiz 3 part A1
    }
}
