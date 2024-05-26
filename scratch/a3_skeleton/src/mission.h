#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <deque>
#include <mutex>
#include <thread>

class Mission : public rclcpp::Node
{

public:
    Mission();

    ~Mission();

    void produceMarkers(geometry_msgs::msg::PoseArray msg, std::string ns, int32_t shape, geometry_msgs::msg::Vector3 size, std_msgs::msg::ColorRGBA colour);

    void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    void conesCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

private:
    void run();//!< Function that will run continously in the thread
    void progress();//!< Function will update of progress of the mission (triggered by the timer)
    double distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt);//<! distance between the current odometry and next goal

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_;//!< Goal Pose subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_;//!< Odometry Pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr conesSub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_; //! Visualisation Marker publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr progressPub_;

    visualization_msgs::msg::MarkerArray markerArray_; //!< Marker Array

    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals

    std::mutex odoMtx_;//<! Mutex for odo
    nav_msgs::msg::Odometry odo_; //!< Storage of odometry 

    std::mutex goalsMtx_;//<! Mutex for goals_ and init_dist_to_goal_;
    double initialDistToGoal_;
    std::deque<geometry_msgs::msg::Point> goals_;
    double distanceToCurrentGoal_;
    unsigned int progress_;

    double tolerance_;

    std::atomic<bool> running_;//<! Indicates if we are chasing (pursuing goal)
};