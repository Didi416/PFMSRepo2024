#include "mission.h"
#include <iostream>

Mission::Mission(): Node("mission"){
    tolerance_ = 0.5;
    running_ = false;
    initialDistToGoal_ = 0;
    progress_ = 0;
    goalProcessed_ = true;
    this->declare_parameter<bool>("_advanced", false);
    this->get_parameter("_advanced", advanced_);
    goalsMsg_ = std::make_shared<geometry_msgs::msg::PoseArray>();
    //Subscribing to laser, this will call the laserCallback function when a new message is published
    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("orange/laserscan", 10, 
                                std::bind(&Mission::laserCallback,this,std::placeholders::_1));   
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Mission::goalsCallback,this,std::placeholders::_1));
    conesSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/cones", 10, 
                                std::bind(&Mission::conesCallback,this,std::placeholders::_1));
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Mission::odomCallback,this,std::placeholders::_1));  
    markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",3); 
    conesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/cones",10);
    goalsPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/goals",10);
    flagPub_ = this->create_publisher<std_msgs::msg::Bool>("orange/flag_mission",3);

    missionService_ = this->create_service<std_srvs::srv::SetBool>("orange/mission", 
                std::bind(&Mission::detectService,this,std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Mission::timerCallback, this));

    thread_ = new std::thread(&Mission::run, this);
}

Mission::~Mission(){
    // if(thread_->joinable()){
    //     thread_->join();
    // }
}

void Mission::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){

    RCLCPP_INFO(this->get_logger(), "Mission Goals Callback");
    std::unique_lock<std::mutex> lck(goalMtx_);
    for (auto pose:msg->poses){
        goals_.push_back(pose.position);
    }
    goalProcessed_ = true;
    locatedGoals.poses.clear();
    goalsMsg_->poses.clear();
    lck.unlock();
    goalCV_.notify_all();

    geometry_msgs::msg::Vector3 size;
    size.x = 0.5; size.y = 0.5; size.z = 0.5;
    std_msgs::msg::ColorRGBA colour;
    colour.a = 1.0; colour.r = 1.0; colour.g = 0.5; colour.b = 0.0;
    produceMarkers(*msg,"goals", 1, size, colour);

}

void Mission::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
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
    std::vector<geometry_msgs::msg::Point> tempCones = laserProcessingPtr_->detectConeCentres();
    geometry_msgs::msg::Pose cone_pose;
    geometry_msgs::msg::PoseArray detected_cones;
    detected_cones.poses.clear();
    for(auto cone:tempCones){
        cone_pose.position = transformPoint(cone);
        detected_cones.poses.push_back(cone_pose);
    }
    
    conesPub_->publish(detected_cones);
    detectRoad(tempCones);
    sortAndRemoveDuplicates();
    if(advanced_){
        std::unique_lock<std::mutex> lck(goalMtx_);
        if(goalCV_.wait_for(lck, std::chrono::milliseconds(50), [this]{ return goalProcessed_;})){
            bool setGoal = true;
            for (size_t i=0;i<road_.size(); i++){
                roadCentre_.position.x = (road_.at(i).first.x + road_.at(i).second.x)/2;
                roadCentre_.position.y = (road_.at(i).first.y + road_.at(i).second.y)/2;
                    for(size_t k=0; k<roadCentres_.size(); k++){
                        if(std::abs(roadCentre_.position.x - roadCentres_.at(k).x) < 2.5 && std::abs(roadCentre_.position.y - roadCentres_.at(k).y) < 2.5){
                            setGoal = false;
                            break;
                        }
                    }
                if(setGoal){
                    locatedGoals.poses.push_back(roadCentre_);
                    roadCentres_.push_back({roadCentre_.position.x, roadCentre_.position.y, roadCentre_.position.z});
                    // std::cout<<"Located goal x: "<<roadCentre_.position.x<<", goal y: "<<roadCentre_.position.y<<std::endl;
                }
                else{setGoal = true;}
            }
            if(locatedGoals.poses.size() != 0){
                goalsPub_->publish(locatedGoals);
                goalsMsg_->poses = locatedGoals.poses;
                goalProcessed_ = false;
            }
        }
        lck.unlock();
    }
}

void Mission::sortAndRemoveDuplicates() {
    // sort the vector based on both elements of the pairs
    std::sort(road_.begin(), road_.end(), [](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& a, const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& b) {
        if ((std::abs(a.first.x - b.first.x) < 1) && (std::abs(a.first.y - b.first.y))) {
            return a.second.x < b.second.x || (std::abs(a.second.x - b.second.x) < 1 && a.second.y < b.second.y);
        } else {
            return a.first.x < b.first.x || (std::abs(a.first.x - b.first.x) < 1 && a.first.y < b.first.y);
        }
    });

    // remove duplicates
    road_.erase(std::unique(road_.begin(), road_.end(), [](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& a, const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& b) {
        return (std::abs(a.first.x - b.first.x) < 1 && std::abs(a.first.y - b.first.y) < 1 && std::abs(a.second.x - b.second.x) < 1 && std::abs(a.second.y - b.second.y) < 1) || 
                (std::abs(a.first.x - b.second.x) < 1 && std::abs(a.first.y - b.second.y) < 1 && std::abs(a.second.x - b.first.x) < 1 && std::abs(a.second.y - b.first.y) < 1);
    }), road_.end());
}

geometry_msgs::msg::Point Mission::transformPoint(geometry_msgs::msg::Point point){
    std::unique_lock<std::mutex> lck(odoMtx_);
    geometry_msgs::msg::Point transformedPoint;
    tf2::Vector3 audiPoint(pose_.position.x, pose_.position.y, pose_.position.z);
    tf2::Quaternion q(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
    lck.unlock();

    tf2::Transform transform(q,audiPoint);
    tf2::Vector3 conePoint(point.x+3.75, point.y, point.z);
    tf2::Vector3 transformedVecPoint = transform * conePoint;
    transformedPoint.x = transformedVecPoint.x();
    transformedPoint.y = transformedVecPoint.y();
    transformedPoint.z = transformedVecPoint.z();

    return transformedPoint;
}

void Mission::detectRoad(std::vector<geometry_msgs::msg::Point> points){
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point pointPair;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> roadPairs;
    double dx;
    double dy;
    double euDist;
    float tolerance = 3.5; //road width tolerance (not exactly 8m)
    double roadWidth = 8; //roughly 8m
    std::pair<std::string, std::string> side;
    bool paired = false;
    for (size_t i=0; i<points.size(); i++){ //Sort into pairs, one cone on either side of the audi, calculated in cars frame
        point = points.at(i);

        for (size_t j=0; j<points.size(); j++){
            pointPair = points.at(j);

            if(std::abs(point.y) < 1){
                if(pointPair.y > 1){
                    side.second = "left";
                    side.first = "right";
                }
                else if(pointPair.y < -1){
                    side.second = "right";
                    side.first = "left";
                }
                paired = true;
            }
            else if(point.y > 0){side.first = "left";}
            else{side.first = "right";}
            if(!paired){
                if(std::abs(pointPair.y) < 1){
                    if(point.y > 1){
                        side.first = "left";
                        side.second = "right";
                    }
                    else if(point.y < -1){
                        side.first = "right";
                        side.second = "left";
                    }
                }
                else if(pointPair.y > 0){side.second = "left";}
                else{side.second = "right";}
            }
            else{paired = false;}

            if(side.first == side.second){ //check for if cones are already paired or if they are one the same side of the road
                continue;
            }
            dx = point.x - pointPair.x; //calculate difference in x
            dy = point.y - pointPair.y; //calculate difference in y
            euDist = std::hypot(dx,dy);
            if (std::abs(roadWidth-euDist) < tolerance && std::hypot(point.x, point.y)<20 && std::hypot(pointPair.x, pointPair.y)<20){
                road_.push_back(std::make_pair(transformPoint(point), transformPoint(pointPair))); //
                break;
            }
        }
    }
}

void Mission::produceMarkers(geometry_msgs::msg::PoseArray msg, std::string ns, int32_t shape, geometry_msgs::msg::Vector3 size, std_msgs::msg::ColorRGBA colour){
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto& pose : msg.poses)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = marker_array.markers.size();
        marker.type = shape;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale = size;
        marker.color = colour;
        marker_array.markers.push_back(marker);

    }
    markerPub_->publish(marker_array);
}

void Mission::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    // RCLCPP_INFO(this->get_logger(), "Mission Odo Callback");
    std::unique_lock<std::mutex> lck(odoMtx_);
    pose_ = msg->pose.pose;
    lck.unlock();  
}

void Mission::conesCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){
    geometry_msgs::msg::Vector3 size;
    size.x = 0.2; size.y = 0.2; size.z = 0.5;
    std_msgs::msg::ColorRGBA colour;
    colour.a = 1.0; colour.r = 1.0; colour.g = 0.5; colour.b = 0.0;
    produceMarkers(*msg,"cones", 3, size, colour);
    // std::cout<<"Cones marked";
}

void Mission::timerCallback(){
    // RCLCPP_INFO(this->get_logger(), "Timer Callback");
    if(goalsMsg_->poses.size() != 0){
        RCLCPP_INFO(this->get_logger(), "timer calls goals callback");
        goalsCallback(goalsMsg_);
    }
}

double Mission::distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt){ 
    return sqrt(std::pow(odo.pose.pose.position.x-pt.x,2) + std::pow(odo.pose.pose.position.y-pt.y,2));
}

// void Mission::progress(){
//     // RCLCPP_INFO(this->get_logger(), "Mission Progress Callback");
//     std_msgs::msg::String progressMsg = std_msgs::msg::String();
//     if(running_){
//         nav_msgs::msg::Odometry odo;
//         geometry_msgs::msg::Point goal;
//         std::unique_lock<std::mutex> lck1(odoMtx_);
//         odo = odo_;
//         lck1.unlock();
//         std::unique_lock<std::mutex> lck2(goalMtx_);
//         goal = goals_.front();
//         lck2.unlock();
//         double dist = distance(odo,goal);
//         progress_ = (unsigned int)(100.0*((initialDistToGoal_ - dist)/initialDistToGoal_));
//     }
//     // RCLCPP_INFO(this->get_logger(), "Mission Progress: " + progress_);
//     progressMsg.data = progress_;
//     progressPub_->publish(progressMsg);
// }

void Mission::run(){
    double dx1, dy1, dx2, dy2;
    std::pair<double, double> euDist;
    float tolerance = 3;
    double estimatedRoadDist = 4;
    bool goalReachable = false;
    double dist;
    nav_msgs::msg::Odometry odo;
    geometry_msgs::msg::Point goal;
    while(rclcpp::ok){
        // RCLCPP_INFO(this->get_logger(), "Mission Run Callback");
        if(goals_.size() > 0){
            running_ = true;
        }
        if(startMission_.data && running_){
            std::unique_lock<std::mutex> lck1(odoMtx_);
            odo = odo_;
            lck1.unlock();
            std::unique_lock<std::mutex> lck2(goalMtx_);
            goal = goals_.front();

            dist = distance(odo, goal);
            if(dist<tolerance_){
                goals_.pop_front();
                if(goals_.size() > 0){                    
                    goal = goals_.front();
                    initialDistToGoal_ = distance(odo,goal);
                }
                else if(goals_.size() == 0){
                    running_ = false;
                }
            }
            lck2.unlock();
        
            for (size_t j=0; j<road_.size(); j++){
                dx1 = road_.at(j).first.x - goal.x;
                dy1 = road_.at(j).first.y - goal.y;
                dx2 = road_.at(j).second.x - goal.x;
                dy2 = road_.at(j).second.y - goal.y;
                euDist.first = std::hypot(dx1,dy1);
                euDist.second = std::hypot(dx2,dy2);
                if(std::abs(estimatedRoadDist - euDist.first) < tolerance && std::abs(estimatedRoadDist - euDist.second) < tolerance){
                    goalReachable = true;
                    break;
                }
            }
            if(goalReachable){
                goalReachable = false;
            }else{
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal cannot be reached, stopping mission.");
                startMission_.data = false;
                flagPub_->publish(startMission_);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Mission::detectService(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res){ 
    startMission_.data = req->data;
    flagPub_->publish(startMission_);
    if(req->data){
        res->message = "Mission Running, Progress: " + progress_;
    }
    else{
        res->message = "Stop Mission, Progress: " + progress_;
    }
    res->success = true;
}