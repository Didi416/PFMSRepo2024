#include "controller.h"

Controller::Controller(): Node("controller") //Node name is "a3_skeleton"
{
    goalsMsg_ = std::make_shared<geometry_msgs::msg::PoseArray>();
    goalProcessed_ = true;
    this->declare_parameter<bool>("_advanced", false);
    this->get_parameter("_advanced", advanced_);
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

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Controller::timerCallback, this));
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
    // std::cout<<"Scan complete \n";
    std::vector<geometry_msgs::msg::Point> tempCones = laserProcessingPtr_->detectConeCentres();
    geometry_msgs::msg::Pose cone_pose;
    geometry_msgs::msg::Pose goal_pose;
    detected_cones_.poses.clear();
    for(auto cone:tempCones){
        cone_pose.position = transformPoint(cone);
        detected_cones_.poses.push_back(cone_pose);
    }
    
    conesPub_->publish(detected_cones_);

    // std::cout<<"temp: "<<tempCones.size()<<" total: "<<road_.size()<<std::endl;
    // visitedCones_.clear();
    // if(totalCones.size() == 0){
    //     totalCones = tempCones;//initial cones from first scan
    // }
    // else{
    // if(road_.size() != 0){
    //     for (size_t i=0;i<tempCones.size(); i++){
    //         for(size_t j=0; j<road_.size(); j++){
    //             if((std::abs(transformPoint(tempCones.at(i)).x - road_.at(j).first.x) < 1 && std::abs(transformPoint(tempCones.at(i)).y - road_.at(j).first.y) < 1) || 
    //                 (std::abs(transformPoint(tempCones.at(i)).x - road_.at(j).second.x) < 1 && std::abs(transformPoint(tempCones.at(i)).y - road_.at(j).second.y) < 1)){ //check if cone already in totalCones vector
    //                 tempCones.erase(tempCones.begin()+i); //remove cone point from tempCones
    //                 // if(visitedCones_.find(j) == visitedCones_.end()){ //check if cone is already visited/marked
    //                 //     visitedCones_.insert(j);
    //                 // }
    //                 i--;
    //                 break;
    //             }
    //         }
    //     }
    //     // totalCones.insert(totalCones.end(), tempCones.begin(), tempCones.end());
    // }

    detectRoad(tempCones);

    // std::sort(road_.begin(), road_.end(), [this](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& p1, 
    //                                           const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& p2) {
    //     if (comparePoints(p1.first, p2.first)) return true;
    //     if (comparePoints(p2.first, p1.first)) return false;
    //     return comparePoints(p1.second, p2.second);
    // });
    // auto last = std::unique(road_.begin(), road_.end(), [](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& p1,
    //                                                        const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& p2) {
    //     return p1.first.x == p2.first.x && p1.first.y == p2.first.y && p1.second.x == p2.second.x && p1.second.y == p2.second.y;
    // });
    // road_.erase(last, road_.end());

    sortAndRemoveDuplicates();

    // geometry_msgs::msg::Point roadGlobalPose1;
    // geometry_msgs::msg::Point roadGlobalPose2;
    // for(size_t i=0; i<road_.size(); i++){
    //     if((road_.at(i).first.x < -5 )|| (road_.at(i).second.x < -5)){
    //         road_.erase(road_.begin()+i);
    //         i--;
    //         continue;
    //     }
    // //     roadGlobalPose1 = transformPoint(road_.at(i).first);
    // //     roadGlobalPose2 = transformPoint(road_.at(i).second);
    // //     transformedRoad_.push_back(std::make_pair(roadGlobalPose1, roadGlobalPose2));
    // }
    
    // std::cout<<"Detected Road \n";
    if(advanced_){
        std::unique_lock<std::mutex> lck(goalMtx_);
        if(goalCV_.wait_for(lck, std::chrono::milliseconds(50), [this]{ return goalProcessed_;})){
            bool setGoal = true;
            for (size_t i=0;i<road_.size(); i++){
                roadCentre_.position.x = (road_.at(i).first.x + road_.at(i).second.x)/2;
                roadCentre_.position.y = (road_.at(i).first.y + road_.at(i).second.y)/2;
                // std::cout<<"first x: "<<road_.at(i).first.x<<", goal y: "<<road_.at(i).first.y<<std::endl;
                // std::cout<<"second x: "<<road_.at(i).second.x<<", goal y: "<<road_.at(i).second.y<<std::endl;
                // for(size_t j=0; j<locatedGoals.size(); j++){
                //     if(std::abs(roadCentre_.position.x - locatedGoals.at(j).x) < 2.5 && std::abs(roadCentre_.position.y - locatedGoals.at(j).y) < 2.5){
                //         setGoal = false;
                //         break;
                //     }
                    for(size_t k=0; k<roadCentres_.size(); k++){
                        if(std::abs(roadCentre_.position.x - roadCentres_.at(k).x) < 2.5 && std::abs(roadCentre_.position.y - roadCentres_.at(k).y) < 2.5){
                            setGoal = false;
                            break;
                        }
                    }
                // }
                if(setGoal){
                    locatedGoals.poses.push_back(roadCentre_);
                    roadCentres_.push_back({roadCentre_.position.x, roadCentre_.position.y, roadCentre_.position.z});
                    std::cout<<"Located goal x: "<<roadCentre_.position.x<<", goal y: "<<roadCentre_.position.y<<std::endl;
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

void Controller::sortAndRemoveDuplicates() {
    // sort the vector based on both elements of the pairs
    std::sort(road_.begin(), road_.end(), [](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& a, const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& b) {
        if ((std::abs(a.first.x - b.first.x) < 1) && (std::abs(a.first.y - b.first.y))) {
            // std::cout<<"test1 \n";
            return a.second.x < b.second.x || (std::abs(a.second.x - b.second.x) < 1 && a.second.y < b.second.y)/* || (std::abs(a.second.x - b.second.x) && std::abs(a.second.y - b.second.y) && a.second.z < b.second.z)*/;
        } else {
            // std::cout<<"test2 \n";
            return a.first.x < b.first.x || (std::abs(a.first.x - b.first.x) < 1 && a.first.y < b.first.y)/* || (a.first.x == b.first.x && a.first.y == b.first.y && a.first.z < b.first.z)*/;
        }
    });

    // remove duplicates
    // std::cout<<"test3 \n";
    road_.erase(std::unique(road_.begin(), road_.end(), [](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& a, const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& b) {
        // return (a.first.x == b.first.x && a.first.y == b.first.y && a.first.z == b.first.z && a.second.x == b.second.x && a.second.y == b.second.y && a.second.z == b.second.z) ||
        //        (a.first.x == b.second.x && a.first.y == b.second.y && a.first.z == b.second.z && a.second.x == b.first.x && a.second.y == b.first.y && a.second.z == b.first.z);
        return (std::abs(a.first.x - b.first.x) < 1 && std::abs(a.first.y - b.first.y) < 1 && std::abs(a.second.x - b.second.x) < 1 && std::abs(a.second.y - b.second.y) < 1) || 
                (std::abs(a.first.x - b.second.x) < 1 && std::abs(a.first.y - b.second.y) < 1 && std::abs(a.second.x - b.first.x) < 1 && std::abs(a.second.y - b.first.y) < 1);
    }), road_.end());
}

// bool Controller::comparePoints(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
//     if (p1.x != p2.x) return p1.x < p2.x;
//     if (p1.y != p2.y) return p1.y < p2.y;
//     return p1.z < p2.z;
// }

// void Controller::normalizePairs(std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>& pairs) {
//     for (auto& pair : pairs) {
//         if (!comparePoints(pair.first, pair.second)) {
//             std::swap(pair.first, pair.second);
//         }
//     }
// }


geometry_msgs::msg::Point Controller::transformPoint(geometry_msgs::msg::Point point){
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

// std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> Controller::detectRoad(std::vector<geometry_msgs::msg::Point> points){
//     geometry_msgs::msg::Point point;
//     geometry_msgs::msg::Point pointPair;
//     std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> roadPairs;
//     std::pair<std::string, std::string> side;
//     double distance1;
//     double distance2;
//     double euDist;
//     for(size_t i=0; i<points.size(); i++){
//         point = points.at(i);
//         if(point.y > 0){side.first = "left";}
//         else{side.first = "right";}
//         distance1 = std::hypot(point.x, point.y);
//         for(size_t j=0; j<points.size(); j++){
//             pointPair = points.at(j);
//             // std::cout<<"Points: "<<point.x<<" "<<pointPair.x<<" "<<point.y<<" "<<pointPair.y<<std::endl;
//             if(pointPair.y > 0){side.second = "left";}
//             else{side.second = "right";}
//             // if (side.first == side.second){
//             //     // std::cout<<"Same side \n";
//             //     continue;
//             // }
//             // std::cout<<"Different side \n";
//             distance2 = std::hypot(pointPair.x, pointPair.y);
//             euDist = std::hypot(point.x - pointPair.x, point.y - pointPair.y);
//             if(distance1 <= 20 && distance2 <= 20 &&  euDist > 5 && euDist < 12) { 
//                 // std::cout<<"Same Side and within range \n";
//                 roadPairs.push_back(std::make_pair(transformPoint(point), transformPoint(pointPair)));
//                 break;
//             }
//         }
//     }
//     return roadPairs;
// }

std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> Controller::detectRoad(std::vector<geometry_msgs::msg::Point> points){
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point pointPair;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> roadPairs;
    double dx;
    double dy;
    double euDist;
    float tolerance = 3.5; //road width tolerance (not exactly 8m)
    double roadWidth = 8; //roughly 8m
    std::pair<std::string, std::string> side;
    std::set<unsigned int> pairedCones;
    bool paired = false;
    // visitedCones_.clear();
    
    for (size_t i=0; i<points.size(); i++){ //Sort into pairs, one cone on either side of the audi, calculated in cars frame
        // if(pairedCones.find(i) != pairedCones.end()){ //check if cone is already part of a pair
        //     // std::cout<<"Already part of a pair \n";
        //     continue; //go to next cone i, next for loop iteration
        // }
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

            if(/*pairedCones.find(j) != pairedCones.end() || */side.first == side.second){ //check for if cones are already paired or if they are one the same side of the road
                continue;
            }
            dx = point.x - pointPair.x; //calculate difference in x
            dy = point.y - pointPair.y; //calculate difference in y
            euDist = std::hypot(dx,dy);
            // std::cout<<"Hi Cone 1: "<<transformPoint(point).x<<" and y: "<<transformPoint(point).y<<std::endl;
            // std::cout<<"Hi Cone 2: "<<transformPoint(pointPair).x<<" and y: "<<transformPoint(pointPair).y<<std::endl;
            // std::cout<<"EuDist: "<<euDist<<" dx: "<<dx<<" dy: "<<dy<<std::endl;
            if (std::abs(roadWidth-euDist) < tolerance && std::hypot(point.x, point.y)<20 && std::hypot(pointPair.x, pointPair.y)<20){
                // std::cout<<"Cone 1: "<<(point).x<<" and y: "<<(point).y<<std::endl;
                // std::cout<<"Cone 2: "<<(pointPair).x<<" and y: "<<(pointPair).y<<std::endl;
                // std::cout<<"Hi Cone 1: "<<transformPoint(point).x<<" and y: "<<transformPoint(point).y<<std::endl;
                // std::cout<<"Hi Cone 2: "<<transformPoint(pointPair).x<<" and y: "<<transformPoint(pointPair).y<<std::endl;
                road_.push_back(std::make_pair(transformPoint(point), transformPoint(pointPair))); //
                pairedCones.insert(i);
                pairedCones.insert(j);
                break;
            }
        }
    }
    std::cout<<"Point Size: "<<points.size()<<"Road Size: "<<road_.size()<<std::endl;
    return roadPairs;
}

void Controller::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    // RCLCPP_INFO(this->get_logger(), "Odo Callback");
    std::unique_lock<std::mutex> lck(odoMtx_);
    pose_ = msg->pose.pose;
    lck.unlock();
}

void Controller::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){
    RCLCPP_INFO(this->get_logger(), "Goals Callback");
    std::unique_lock<std::mutex> lck(goalMtx_);
    setGoals(*msg);
    std::cout<<"goal size: "<<pfmsGoals_.size()<<std::endl;
    goalProcessed_ = true;
    locatedGoals.poses.clear();
    goalsMsg_->poses.clear();
    lck.unlock();
    goalCV_.notify_all();
}

void Controller::progressCallback(const std::shared_ptr<std_msgs::msg::String> msg){
    progress_ = msg->data;
}

void Controller::setGoals(geometry_msgs::msg::PoseArray msg){
    for (size_t i=0; i<msg.poses.size(); i++){
        pfmsGoals_.push_back({msg.poses.at(i).position.x, msg.poses.at(i).position.y, msg.poses.at(i).position.z});
    }
}

void Controller::timerCallback(){
    // RCLCPP_INFO(this->get_logger(), "Timer Callback");
    if(goalsMsg_->poses.size() != 0){
        RCLCPP_INFO(this->get_logger(), "timer calls goals callback");
        goalsCallback(goalsMsg_);
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