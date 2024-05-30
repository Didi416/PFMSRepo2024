#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "pfms_types.h"
#include "laserprocessing.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <string>
#include <deque>
#include <mutex>
#include <thread>

class Mission : public rclcpp::Node
{

public:
    Mission();

    ~Mission();

private:
    void produceMarkers(geometry_msgs::msg::PoseArray msg, std::string ns, int32_t shape, geometry_msgs::msg::Vector3 size, std_msgs::msg::ColorRGBA colour);

    void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    void conesCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    void timerCallback();

    void run();//!< Function that will run continously in the thread
    void progress();//!< Function will update of progress of the mission (triggered by the timer)
    double distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt);//<! distance between the current odometry and next goal
    geometry_msgs::msg::Point transformPoint(geometry_msgs::msg::Point points);

    void detectRoad(std::vector<geometry_msgs::msg::Point> points);

    /*! @brief LaserScan Callback
    *
    *  @param std::shared_ptr<sensor_msgs::msg::LaserScan - The laserscan message as a const pointer
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    void sortAndRemoveDuplicates();

    /*! @brief - A callback for the service
    *  
    * @param[in] req - The request object
    * @param[out] res - The response object which contains a number of valid readings in last laser scan received "Readings: <number of readings>"
    * if no laser scan is received, the response will contain error message "ERROR: No laser data available"
    */
    void detectService(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;//!< Pointer to the laser scan subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_;//!< Goal Pose subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_;//!< Odometry Pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr conesSub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_; //! Visualisation Marker publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flagPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr conesPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goalsPub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr missionService_;

    std::unique_ptr<LaserProcessing> laserProcessingPtr_;//!< Pointer to the laser processing object
    geometry_msgs::msg::Pose roadCentre_;
    geometry_msgs::msg::PoseArray locatedGoals;
    std::deque<pfms::geometry_msgs::Point> roadCentres_;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> road_;

    std::condition_variable goalCV_;
    bool goalProcessed_;
    std::shared_ptr<geometry_msgs::msg::PoseArray> goalsMsg_;

    visualization_msgs::msg::MarkerArray markerArray_; //!< Marker Array

    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals

    std::mutex odoMtx_;//<! Mutex for odo
    nav_msgs::msg::Odometry odo_; //!< Storage of audi car odometry 
    geometry_msgs::msg::Pose pose_; //! Storage for audi car pose

    std::mutex goalMtx_;//<! Mutex for goals_ and init_dist_to_goal_;
    double initialDistToGoal_;
    std::deque<geometry_msgs::msg::Point> goals_;
    double distanceToCurrentGoal_;
    unsigned int progress_;

    double tolerance_;
    bool advanced_;
    std_msgs::msg::Bool startMission_;

    std::atomic<bool> running_;//<! Indicates if we are chasing (pursuing goal)
};