#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "pfms_types.h"
#include "laserprocessing.h"
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


    /*! @brief - A callback for the service
    *  
    * @param[in] req - The request object
    * @param[out] res - The response object which contains a number of valid readings in last laser scan received "Readings: <number of readings>"
    * if no laser scan is received, the response will contain error message "ERROR: No laser data available"
    */
    // void detect(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    pfms::nav_msgs::Odometry getOdometry(void);

    /**
     * @brief Retrieves value for distance to be travelled to reach current goal, updates as the platform moves to current goal
     * @return distance to be travelled to goal [m]
    */
    void setGoals(geometry_msgs::msg::PoseArray msg);

    virtual void reachGoals(void) = 0;

    double distanceToGoal();

    geometry_msgs::msg::Point transformPoint(geometry_msgs::msg::Point points);

    void detectService(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> detectRoad(std::vector<geometry_msgs::msg::Point> points);

    void timerCallback();

protected:
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
    void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    void progressCallback(const std::shared_ptr<std_msgs::msg::String> msg);

    bool comparePoints(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    void normalizePairs(std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>& pairs);

    void sortAndRemoveDuplicates();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;//!< Pointer to the laser scan subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr progressSub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr conesPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goalsPub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr missionService_;

    std::vector<geometry_msgs::msg::Point> totalCones;
    int prevTotalConeSize_;
    geometry_msgs::msg::PoseArray detected_cones_;
    std::set<unsigned int> visitedCones_;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> road_;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> transformedRoad_;
    
    std::deque<pfms::geometry_msgs::Point> pfmsGoals_;
    double distanceToCurrentGoal_;

    geometry_msgs::msg::Pose roadCentre_;
    geometry_msgs::msg::PoseArray locatedGoals;
    std::deque<pfms::geometry_msgs::Point> roadCentres_;
    std::string progress_;

    std::unique_ptr<LaserProcessing> laserProcessingPtr_;//!< Pointer to the laser processing object
    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals


    geometry_msgs::msg::Pose pose_;
    pfms::nav_msgs::Odometry currentOdo_;
    std::mutex odoMtx_;
    std::mutex goalMtx_;
    std::condition_variable goalCV_;
    bool goalProcessed_;
    std::shared_ptr<geometry_msgs::msg::PoseArray> goalsMsg_;

    bool startMission_;
    bool advanced_;
};