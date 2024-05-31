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

/*!
 *  \brief     Mission Class
 *  \details
 *  Includes all functions needed to calculate goals and dictate mission path
 *  \author    Dyandra Prins
 *  \date      2024-31-05
 */

class Mission : public rclcpp::Node
{

public:
    /*! @brief Mission constructor.
    *
    *  Will initialise the callbacks and internal variables
    */
        Mission();

    /*! @brief Mission destructor.
    *
    *  Will tear down the object
    */
        ~Mission();

private:

    /*! @brief Produces markers for visualisation and publishes them to visualization topic
    *  @param msg - Pose array of points to be visualised in simulation 
    * @param ns - namespace for visualisation points (int32_t)
    * @param shape - shape of visualisation marker as shown in simulation
    * @param size - size of marker (geometry_msgs::msg::Vector3)
    * @param colour - colour or marker (std_msgs/msg/ColorRGBA)
    */
    void produceMarkers(geometry_msgs::msg::PoseArray msg, std::string ns, int32_t shape, geometry_msgs::msg::Vector3 size, std_msgs::msg::ColorRGBA colour);
    
    /*! @brief Goals Callback, subscribes to goals topic
    *
    *  @param std::shared_ptr<geometry_msgs::msg::PoseArray - The goals message as a const pointer of pose array (locations)
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);
    
    /*! @brief Odometry Callback, subscribes odometery topic
    *
    *  @param std::shared_ptr<nav_msgs::msg::Odometry - The odometry message as a const pointer of odometry data type
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /*! @brief Cones Callback, subscribes to cones topic
    *
    *  @param std::shared_ptr<geometry_msgs::msg::PoseArray - The cones message as a const pointer of pose array (locations)
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void conesCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    /*! @brief Timer Callback, called from wall_timer and is used to ensure goals are subscribed to when a message comes throug
    */
    void timerCallback();

     /*!
     * @brief Runs the mission continuously in thread until node shutdown
     */
    void run();

    /*!
     * @brief Will update of progress of the mission (triggered by a timer)
    */
    void progress();
    /*!
     * @brief Calculated distance to current goal, to be used in progress updates
     @return distance from current odometry to next goal
    */
    double distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt);
    /*!
     * @brief Transforms points in local frame to global frame
     @return geometry_msg::msg::Point of transformed input point
    */
    geometry_msgs::msg::Point transformPoint(geometry_msgs::msg::Point point);

    /*! @brief LaserScan Callback
    *
    *  @param std::shared_ptr<sensor_msgs::msg::LaserScan - The laserscan message as a const pointer
    *  @note This function and the declaration are ROS specific to handle callbacks
    */
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    /*!
     * @brief Sorts through road pairs vector and removes any duplicate pairs
    */
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
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr conesSub_; //!< Cones location subscriber

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_; //!< Visualisation Marker publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flagPub_; //!< Mission flag (to stop/start mission) publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr conesPub_; //!< Cones location publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goalsPub_; //!< Goals location publisher

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr missionService_; //!< Service Server object to start/stop mission

    std::unique_ptr<LaserProcessing> laserProcessingPtr_;//!< Pointer to the laser processing object
    geometry_msgs::msg::Pose roadCentre_; //!< Pose for current road centre (used with road pairs)
    geometry_msgs::msg::PoseArray locatedGoals; //!< Pose Array for newly located goals, to be sent through goals publisher
    std::deque<pfms::geometry_msgs::Point> roadCentres_; //!< storage for all currently located road centres (for ensuring no doubling of goals)
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> road_; //!< storage for road pairs

    std::condition_variable goalCV_; //!< conditional variable for access to goals data, used to ensure goals are subscribed to before searching for new goals
    bool goalProcessed_;//!< boolean to represent if goals have been processed by subscriber before searching or new goals, used with conditional variable

    visualization_msgs::msg::MarkerArray markerArray_; //!< Visualisation Marker Array

    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to call goals subscriber if goalsCallback doesn't catch the published message
    std::shared_ptr<geometry_msgs::msg::PoseArray> goalsMsg_; //!< storage for current published goals message, used to ensure goals are subscribed to before searching for new goals

    std::mutex odoMtx_;//!< Mutex for secure odo data access
    nav_msgs::msg::Odometry odo_; //!< Storage of audi car odometry 
    geometry_msgs::msg::Pose pose_; //!< Storage for audi car pose

    std::mutex goalMtx_;//!< Mutex for goals_ and initialDistToGoal_;
    double initialDistToGoal_; //!< storage for initial distance to next goal (from previous goal)
    double totalMissionDistance_;
    std::deque<geometry_msgs::msg::Point> goals_; //!< Local storage of goals to move to
    unsigned int progress_; //!< storage for mission progress

    double tolerance_; //!< storage for acceptable goal tolerance
    bool advanced_; //!< boolean indicator if advanced section is activated
    bool initialOdoTaken_; //!< boolean indicating if odometry has been taken on startup
    std_msgs::msg::Bool startMission_; //!< std_msgs::msg::Bool to send over flag publisher and mission service to stop/start mission

    std::atomic<bool> running_;//!< Indicates if we are chasing (pursuing goal)
};