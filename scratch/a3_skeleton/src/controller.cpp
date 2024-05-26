#include "controller.h"

Controller::Controller(): Node("controller") //Node name is "a3_skeleton"
{
    advanced_ = false;
    //Subscribing to laser, this will call the laserCallback function when a new message is published
    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("orange/laserscan", 10, 
                                std::bind(&Controller::laserCallback,this,std::placeholders::_1));   
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Controller::odomCallback,this,std::placeholders::_1));   
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Controller::goalsCallback,this,std::placeholders::_1));
    progressSub_ = this->create_subscription<std_msgs::msg::String>("orange/progress", 10, 
                                std::bind(&Controller::progressCallback,this,std::placeholders::_1));    
    goalsPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/goals",10);                         
    markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10); 
    conesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/cones",10);  
    missionService_ = this->create_service<std_srvs::srv::SetBool>("orange/mission", 
                std::bind(&Controller::detectService,this,std::placeholders::_1, std::placeholders::_2));
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
    for(auto cone:cones){
        cone_pose.position = transformPoint(cone);
        detected_cones_.poses.push_back(cone_pose);
    }
    
    conesPub_->publish(detected_cones_);
    // std::cout<<"Cones size: "<<cones.size()<<std::endl;
    // road_.first = transformPoint(laserProcessingPtr_->detectRoad().first);
    // road_.second = transformPoint(laserProcessingPtr_->detectRoad().second);

    road_ = detectRoad(cones);
    // std::cout<<"Direct Road Size: "<<road_.size()<<std::endl;
    // if(advanced_){
    //     roadCentres_.poses.clear();
    //     for (size_t i=0;i<road_.size(); i++){
    //         roadCentre_.position.x = (road_.at(i).first.x + road_.at(i).second.x)/2;
    //         roadCentre_.position.y = (road_.at(i).first.y + road_.at(i).second.y)/2;
    //         for(size_t j=0; j<pfmsGoals_.size(); j++){
    //             if(std::abs(roadCentre_.position.x - pfmsGoals_.at(j).x) < 1 && std::abs(roadCentre_.position.y - pfmsGoals_.at(j).y) < 1){
    //                 roadCentres_.poses.push_back(roadCentre_);
    //             }
    //         }
    //     }
    //     goalsPub_->publish(roadCentres_);
    // }
}

geometry_msgs::msg::Point Controller::transformPoint(geometry_msgs::msg::Point point){
    geometry_msgs::msg::Point transformedPoint;
    tf2::Vector3 audiPoint(pose_.position.x, pose_.position.y, pose_.position.z);
    tf2::Quaternion q(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
    tf2::Transform transform(q,audiPoint);
    tf2::Vector3 conePoint(point.x+3.75, point.y, point.z);
    tf2::Vector3 transformedVecPoint = transform * conePoint;
    transformedPoint.x = transformedVecPoint.x();
    transformedPoint.y = transformedVecPoint.y();
    transformedPoint.z = transformedVecPoint.z();

    return transformedPoint;
}

std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> Controller::detectRoad(std::vector<geometry_msgs::msg::Point> points){
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point pointPair;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> roadPairs;
    double distance1;
    double distance2;
    double euDist;
    for(size_t i=0; i<points.size(); i++){
        point = points.at(i);
        distance1 = std::hypot(point.x, point.y);
        for(size_t j=0; j<points.size(); j++){
            pointPair = points.at(j);
            distance2 = std::hypot(pointPair.x, pointPair.y);
            euDist = std::hypot(point.x - pointPair.x, point.y - pointPair.y);
            if(distance1 <= 20 && distance2 <= 20 &&  euDist > 5 && euDist < 13 ) { 
                roadPairs.push_back(std::make_pair(transformPoint(point), transformPoint(pointPair)));
                break;
            }
        }
    }
    return roadPairs;
}

// std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> Controller::detectRoad(std::vector<geometry_msgs::msg::Point> points){

//     geometry_msgs::msg::Point point;
//     geometry_msgs::msg::Point pointPair;
//     std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> roadPairs;

//     double dx;
//     double dy;
//     double euDist;
//     float tolerance = 3; //road width tolerance (not exactly 8m)
//     double roadWidth = 8; //roughly 8m

//     std::set<unsigned int> visitedCones;
//     std::pair<std::string, std::string> side;
//     // std::cout<<"Start \n";

//     for (size_t i=0; i<points.size(); i++){ //Sort into pairs, one cone on either side of the audi, calculated in cars frame
//         // if(visitedCones.find(i) != visitedCones.end()){ //check if cone is already part of a pair
//         //     // std::cout<<"Already part of a pair \n";
//         //     continue; //go to next cone i, next for loop iteration
//         // }
//         point = points.at(i);
//         if(point.y > 0){side.first = "left";}
//         else{side.first = "right";}
//         for (size_t j=0; j<points.size(); j++){
//             pointPair = points.at(j);
//             if(pointPair.y > 0){side.second = "left";}
//             else{side.second = "right";}
//             // if(visitedCones.find(j) != visitedCones.end()){
//             //     continue;
//             // }
//             // if (side.first == side.second){
//             //     continue;
//             // }
//             dx = point.x - pointPair.x;
//             dy = point.y - pointPair.y;
//             euDist = std::hypot(dx,dy);
//             if (std::abs(roadWidth-euDist) < tolerance){
//                 // std::cout<<"EuDist: "<<euDist<<" dx: "<<dx<<" dy: "<<dy<<std::endl;
//                 roadPairs.push_back(std::make_pair(transformPoint(point), transformPoint(pointPair)));
//                 // visitedCones.insert(i);
//                 // visitedCones.insert(j);
//                 break;
//             }
//         }
//     }
//     return roadPairs;
// }

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

void Controller::detectService(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res){ 
    startMission_ = req->data;
    if(req->data){
        res->message = "Mission Running, Progress: " + progress_;
    }
    else{
        res->message = "Stop Mission, Progress: " + progress_;
    }
    res->success = true;
}