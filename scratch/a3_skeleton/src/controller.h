#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


#include "laserprocessing.h"

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


    /*! @brief - A callback for the service
    *  
    * @param[in] req - The request object
    * @param[out] res - The response object which contains a number of valid readings in last laser scan received "Readings: <number of readings>"
    * if no laser scan is received, the response will contain error message "ERROR: No laser data available"
    */
    // void detect(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    geometry_msgs::msg::Pose getOdometry(void);

    virtual void reachGoals(void) = 0;

    private:
    geometry_msgs::msg::PoseArray goals_;

    /*! @brief LaserScan Callback
    *
    *  @param std::shared_ptr<sensor_msgs::msg::LaserScan - The laserscan message as a const pointer
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

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
    void goalsCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; //!< Pointer to the service object 

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;//!< Pointer to the laser scan subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_;

    std::unique_ptr<LaserProcessing> laserProcessingPtr_;//!< Pointer to the laser processing object
    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals

    geometry_msgs::msg::Pose pose_;
    std::mutex poseMtx_;
    std::mutex odoMtx_;

};