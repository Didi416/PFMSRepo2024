#include "controller.h"

Controller::Controller(): Node("controller") //Node name is "a3_skeleton"
{
    //Subscribing to laser, this will call the laserCallback function when a new message is published
    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("orange/laserscan", 10, 
                                std::bind(&Controller::laserCallback,this,std::placeholders::_1));   
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Controller::odomCallback,this,std::placeholders::_1));   
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Controller::goalsCallback,this,std::placeholders::_1));
    progressSub_ = this->create_subscription<std_msgs::msg::String>("orange/progress", 10, 
                                std::bind(&Controller::progressCallback,this,std::placeholders::_1));                            
    markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",3); 
    conesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/cones",3);  
    missionService_ = this->create_service<std_srvs::srv::SetBool>("orange/mission", 
                std::bind(&Controller::detect,this,std::placeholders::_1, std::placeholders::_2));
}

Controller::~Controller()
{
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
    std::vector<geometry_msgs::msg::Point> cones = laserProcessingPtr_->detectConeCentres();
    geometry_msgs::msg::Pose cone_pose;
    
    detected_cones_.poses.clear();
    
    for (auto cone:cones){
        tf2::Vector3 audiPoint(pose_.position.x, pose_.position.y, pose_.position.z);
        tf2::Quaternion q(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
        tf2::Transform transform(q,audiPoint);
        tf2::Vector3 conePoint(cone.x+3.75, cone.y, cone.z);
        tf2::Vector3 transformedConePoint = transform * conePoint;
        cone_pose.position.x = transformedConePoint.x();
        cone_pose.position.y = transformedConePoint.y();
        cone_pose.position.z = transformedConePoint.z();
        detected_cones_.poses.push_back(cone_pose);
    }
    conesPub_->publish(detected_cones_);
    produceMarkers(detected_cones_);
}

void Controller::produceMarkers(geometry_msgs::msg::PoseArray msg){
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto& pose : detected_cones_.poses)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "cones";
        marker.id = marker_array.markers.size();
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);

    }
    markerPub_->publish(marker_array);
}

void Controller::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    // RCLCPP_INFO(this->get_logger(), "Odo Callback");
    std::unique_lock<std::mutex> lck(odoMtx_);
    pose_ = msg->pose.pose;
    lck.unlock();
}

void Controller::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){
    RCLCPP_INFO(this->get_logger(), "Goals Callback");
    setGoals(*msg);
}

void Controller::progressCallback(const std::shared_ptr<std_msgs::msg::String> msg){
    progress_ = msg->data;
}

void Controller::setGoals(geometry_msgs::msg::PoseArray msg){
    for (size_t i=0; i<msg.poses.size(); i++){
        pfmsGoals_.push_back({msg.poses.at(i).position.x, msg.poses.at(i).position.y, msg.poses.at(i).position.z});
    }
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){
    std::unique_lock<std::mutex> lck(odoMtx_);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current Odo: " << pose_.position.x << ", " << pose_.position.y);
    currentOdo_.position.x = pose_.position.x;
    currentOdo_.position.y = pose_.position.y;
    currentOdo_.position.z = pose_.position.z;
    currentOdo_.yaw = tf2::getYaw(pose_.orientation);
    lck.unlock();
    return currentOdo_;
}

double Controller::distanceToGoal(void){ // Returns the distance to current goal as calculated in checkOriginToDestinatione(), continuously updated through reachGoals
    return distanceToCurrentGoal_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}

void Controller::detect(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res){ 
    startMission_ = req->data;
    if(req->data){
        res->message = "Mission Running, Progress: " + progress_;
    }
    else{
        res->message = "Stop Mission, Progress: " + progress_;
    }
    res->success = true;
}