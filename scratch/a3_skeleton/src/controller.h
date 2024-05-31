#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pfms_types.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/*!
 *  \brief     Controller Class
 *  \details
 *  Abstact base class for Ackerman (can be for other platforms as well) includes implementing functions with common programming
 * and including virtual functions for those needing to be implemented in derived classes due to differing programming requirements.
 * listens to topics published to by Mission and setting goals for platofrm and starting/stopping mission accordingly.
 *  \author    Dyandra Prins
 *  \date      2024-02-05
 */

class Controller: public rclcpp::Node{

public:
    /*! @brief Controller constructor. Will initialise the callbacks and internal variables
    */
        Controller();

    /*! @brief Controller destructor. Will tear down the object
    */
    ~Controller();

protected:
    /*! @brief Odometry Callback, subscribes odometery topic
    *
    *  @param std::shared_ptr<nav_msgs::msg::Odometry - The odometry message as a const pointer of odometry data type
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /*! @brief Goals Callback, subscribes to goals topic
    *
    *  @param std::shared_ptr<geometry_msgs::msg::PoseArray - The goals message as a const pointer of pose array
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    /*! @brief Flag Callback, subscribes to flag_mission topic, listening for if mission/platform needs to be stopped
    *
    *  @param std::shared_ptr<std_,msgs::msg::Bool - The flag message as a const pointer of boolean
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void flagCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);

    /*! @brief Sets goals as received by goals subscriber
        @sa goalsCallback
    *
    *  @param geometry_msgs::msg::PoseArray takes in pose array of new goals sent over goals topic
    */
    void setGoals(geometry_msgs::msg::PoseArray msg);

    /*! @brief Movement to Reach goal - execute control to reach goal, called in thread, so non-blocking
    */
    virtual void reachGoals(void) = 0;

    /*! @brief Returns current odometry information from platform
    @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
    */
    pfms::nav_msgs::Odometry getOdometry(void);
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_; //!< Subscriber to platform odometry topic
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_; //!< Subscriber to goals topic
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flagSub_; //!< Subscriber to flag_mission topic
    
    std::deque<pfms::geometry_msgs::Point> pfmsGoals_; //!< Local storage of goals to move to, uses pfms types to be able to use audi library

    std::thread* thread_; //!< Thread object pointer for reachGoals

    geometry_msgs::msg::Pose pose_; //!< Pose of platform odometry
    pfms::nav_msgs::Odometry currentOdo_; //!< Pfms type odometry for calculations with audi library
    std::mutex odoMtx_; //!< odometry mutex for secure data access

    bool startMission_; //!< boolean to indicat whether mission should run or not
    double tolerance_; //!< storage for acceptable tolerance to goals
};