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

class Controller: public rclcpp::Node{

public:
    /*! @brief Sample constructor.
    *
    *  Will initialise the callbacks and internal variables
    */
        Controller();

    /*! @brief Sample destructor.
    *
    *  Will tear down the object
    */
    ~Controller();

protected:
    /*! @brief Odometry Callback
    *
    *  @param std::shared_ptr<nav_msgs::msg::Odometry - The odometry message as a const pointer
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /*! @brief Goals Callback
    *
    *  @param std::shared_ptr<geometry_msgs::msg::PoseArray - The goals message as a const pointer
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    void flagCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);

    void setGoals(geometry_msgs::msg::PoseArray msg);

    virtual void reachGoals(void) = 0;

    pfms::nav_msgs::Odometry getOdometry(void);
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flagSub_;
    
    std::deque<pfms::geometry_msgs::Point> pfmsGoals_;

    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals


    geometry_msgs::msg::Pose pose_;
    pfms::nav_msgs::Odometry currentOdo_;
    std::mutex odoMtx_;

    bool startMission_;

};