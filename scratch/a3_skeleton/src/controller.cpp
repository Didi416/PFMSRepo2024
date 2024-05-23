#include "controller.h"

Controller::Controller()
    : Node("a3_skeleton") //Node name is "a3_skeleton"
{
    //Subscribing to laser, this will call the laserCallback function when a new message is published
    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("orange/laserscan", 10, 
                                std::bind(&Controller::laserCallback,this,std::placeholders::_1));   

    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Controller::odomCallback,this,std::placeholders::_1));   

    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Controller::goalsCallback,this,std::placeholders::_1));
    
    markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",3);  

    conesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/cones",3);  

    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Controller::goalsCallback, this, std::placeholders::_1));

    // The service is of type std_srvs::srv::Trigger
    // The service is created with the name "detect"
    // The callback function is detect
    // Callback function is called when the service is called
    // Change the service type and callback function to match the requirements of assessment task
    // service_ = this->create_service<std_srvs::srv::Trigger>("detect", 
    //             std::bind(&Sample::detect,this,std::placeholders::_1, std::placeholders::_2));

}

Controller::~Controller()
{
    // We join the thread here to make sure it is finished before the object is destroyed
    // Thanksfull, we check ros::ok() in the threadFunction to make sure it terminates, otherwise
    // we would have a deadlock here as the thread would be waiting for threadFunction to finish
    thread_->join();
}


void Controller::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
    // RCLCPP_INFO(this->get_logger(), "Laser Callback");
   /**
   * This callback creates a new laserProcessingPtr object if it does not exist
   * or updates the scan in the existing object
   */
    //If we have not created the laserProcessingPtr object, create it
    if(laserProcessingPtr_ == nullptr){
        laserProcessingPtr_ = std::make_unique<LaserProcessing>(*msg);
    }
    else{
        laserProcessingPtr_->newScan(*msg);
    }

    // std::vector<geometry_msgs::msg::Point> cones = laserProcessingPtr_->detectConeCentres();
    // geometry_msgs::msg::Pose cone_pose;
    
    // for (auto cone:cones){
    //     cone_pose.position.x = cone.x + pose_.position.x + 3.5; 
    //     cone_pose.position.y = cone.y + pose_.position.y;
    //     detected_cones_.poses.push_back(cone_pose);
    // }
    // conesPub_->publish(detected_cones_);

    // visualization_msgs::msg::MarkerArray marker_array;
    // for (const auto& cone_pose : detected_cones_.poses)
    // {

    //     visualization_msgs::msg::Marker marker;
    //     marker.header.frame_id = "world";
    //     marker.header.stamp = this->now();
    //     marker.ns = "cones";
    //     marker.id = marker_array.markers.size();
    //     marker.type = visualization_msgs::msg::Marker::CYLINDER;
    //     marker.action = visualization_msgs::msg::Marker::ADD;
    //     marker.pose = cone_pose;
    //     marker.scale.x = 0.2;
    //     marker.scale.y = 0.2;
    //     marker.scale.z = 0.5;
    //     marker.color.a = 1.0;
    //     marker.color.r = 1.0;
    //     marker.color.g = 0.5;
    //     marker.color.b = 0.0;
    //     marker_array.markers.push_back(marker);

    // }
    // markerPub_->publish(marker_array);
}

void Controller::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    // RCLCPP_INFO(this->get_logger(), "Odo Callback");
    getOdometry();
    std::unique_lock<std::mutex> lck(poseMtx_);
    pose_ = msg->pose.pose;
}

void Controller::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){

    RCLCPP_INFO(this->get_logger(), "Goals Callback");
    goals_ = *msg;
    for (size_t i=0; i<goals_.poses.size(); i++){
        pfmsGoals_.push_back({goals_.poses.at(i).position.x, goals_.poses.at(i).position.y, goals_.poses.at(i).position.z});
    }

}

pfms::nav_msgs::Odometry Controller::getOdometry(void){
    std::unique_lock<std::mutex> lck(odoMtx_);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current Odo: " << pose_.position.x << ", " << pose_.position.y);
    currentOdo_.position.x = pose_.position.x;
    currentOdo_.position.y = pose_.position.y;
    currentOdo_.position.z = pose_.position.z;
    currentOdo_.yaw = tf2::getYaw(pose_.orientation);
    return currentOdo_;
}

double Controller::distanceToGoal(void){ // Returns the distance to current goal as claculated in checkOriginToDestinatione(), continuously updated through reachGoals
    return distanceToCurrentGoal_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}