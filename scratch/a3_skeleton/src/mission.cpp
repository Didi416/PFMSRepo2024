#include "mission.h"
#include <iostream>

Mission::Mission(): Node("mission"){
    //initialise default global variables upon creation of node
    tolerance_ = 0.5;
    running_ = false;
    initialDistToGoal_ = 0;
    initialOdoTaken_ = false;
    progress_ = 0;
    goalProcessed_ = true;
    //for paramater passed through command line
    this->declare_parameter<bool>("_advanced", false); //set default to false
    this->get_parameter("_advanced", advanced_); //read parameter from command line
    goalsMsg_ = std::make_shared<geometry_msgs::msg::PoseArray>();
    //Subscribing to laser, goals, cones and odometry, will call the corresponding callback function when a new message is published to the topics
    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("orange/laserscan", 10, 
                                std::bind(&Mission::laserCallback,this,std::placeholders::_1));   
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Mission::goalsCallback,this,std::placeholders::_1));
    conesSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/cones", 10, 
                                std::bind(&Mission::conesCallback,this,std::placeholders::_1));
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Mission::odomCallback,this,std::placeholders::_1));  
    
    markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",3); //create publisher for visualisation markers
    conesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/cones",10); //create publihser for cones locations
    goalsPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/goals",10); //create publisher for goals locations
    flagPub_ = this->create_publisher<std_msgs::msg::Bool>("orange/flag_mission",3); //create publisher for start/stop mission flag

    //create a service to activate mission or stop mission upom calling in command line (based on data value)
    missionService_ = this->create_service<std_srvs::srv::SetBool>("orange/mission", 
                std::bind(&Mission::detectService,this,std::placeholders::_1, std::placeholders::_2));

    //create a wall timer to calll function periodically
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Mission::timerCallback, this));
    thread_ = new std::thread(&Mission::run, this); //start thread for Mission::run function (continuously running until node killed)
}

Mission::~Mission(){
    // if(thread_->joinable()){
    //     thread_->join();
    // }
}

void Mission::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){

    RCLCPP_INFO(this->get_logger(), "Mission Goals Callback");
    std::unique_lock<std::mutex> lck(goalMtx_); //lock access to data
    for (auto pose:msg->poses){
        goals_.push_back(pose.position); //add goals to mission goals vector
    }
    goalProcessed_ = true; //set boolean to processed so laser scan can calculate any new goals
    //clear vectors for new goals
    locatedGoals.poses.clear();
    goalsMsg_->poses.clear();
    lck.unlock(); //unlock access for other threads
    goalCV_.notify_all();
    //design visualisation marker (size, shape, colour)
    geometry_msgs::msg::Vector3 size;
    size.x = 0.5; size.y = 0.5; size.z = 0.5;
    std_msgs::msg::ColorRGBA colour;
    colour.a = 1.0; colour.r = 1.0; colour.g = 0.5; colour.b = 0.0;
    //send message under namespace "cones", shape number 3 is CYLINDER
    produceMarkers(*msg,"goals", 1, size, colour);

}

void Mission::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
    // RCLCPP_INFO(this->get_logger(), "Laser Callback");
    //creates a new laserProcessingPtr object if it does not exist or updates the scan in the existing object
    if(laserProcessingPtr_ == nullptr){
        laserProcessingPtr_ = std::make_unique<LaserProcessing>(*msg);
    }
    else{
        laserProcessingPtr_->newScan(*msg);
    }
    //get cone centres from laser processing
    std::vector<geometry_msgs::msg::Point> tempCones = laserProcessingPtr_->detectConeCentres();
    //initialise temporary variables
    geometry_msgs::msg::Pose cone_pose;
    geometry_msgs::msg::PoseArray detected_cones;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> roadPairs; 
    //test for obstructions that aren't cones (firetruck)
    if(laserProcessingPtr_->detectLargeObstacle()){
        RCLCPP_INFO_STREAM(this->get_logger(), "Large object detected, stopping mission");
        startMission_.data = false; //large obstruction in path stops mission
        flagPub_->publish(startMission_); //publish flag update
    }
    if(initialOdoTaken_){ //if initial odometry has been taken (for transforming points)
        //transform all cones in laser scan to send to visualisation markers
        for(auto cone:tempCones){
            cone_pose.position = transformPoint(cone); //tranform points to global frame
            detected_cones.poses.push_back(cone_pose); //pose array to send as message
        }
        
        conesPub_->publish(detected_cones); //publish cones locations
        roadPairs = laserProcessingPtr_->detectRoad(tempCones); //detect road pairs of track
        //transform the points of the road pairs
        for(size_t i=0; i<roadPairs.size(); i++){
            road_.push_back(std::make_pair(transformPoint(roadPairs.at(i).first), transformPoint(roadPairs.at(i).second)));
        }
        //sort through road_ vector and remove any duplicate pairs
        sortAndRemoveDuplicates();

        //__________________________ADVANCED______________________________
        if(advanced_){ //check if advanced parameter is set
            std::unique_lock<std::mutex> lck(goalMtx_); //lock goalMtx for secure data access
            if(goalCV_.wait_for(lck, std::chrono::milliseconds(50), [this]{ return goalProcessed_;})){ //wait until goalsCallback has processed previous set of goals
                bool setGoal = true; //assume to set goal everytime
                // std::cout<<"Road Size: "<<road_.size()<<std::endl;
                for (size_t i=0;i<road_.size(); i++){//iterate through road pairs
                    //calculate road centre as goal (midpoint of pair)
                    roadCentre_.position.x = (road_.at(i).first.x + road_.at(i).second.x)/2;
                    roadCentre_.position.y = (road_.at(i).first.y + road_.at(i).second.y)/2;
                    //check for if current road centre has already been processed as a goal
                    for(size_t j=0; j<roadCentres_.size(); j++){
                        if(std::abs(roadCentre_.position.x - roadCentres_.at(j).x) < 5 && std::abs(roadCentre_.position.y - roadCentres_.at(j).y) < 5){
                            setGoal = false; //if already in vector, do not set as new goal
                            break;
                        }
                    }
                    if(setGoal){
                        locatedGoals.poses.push_back(roadCentre_); //add current road centre to newly  located goals
                        roadCentres_.push_back({roadCentre_.position.x, roadCentre_.position.y, roadCentre_.position.z}); //add to already located road centres
                        //display new goal location
                        RCLCPP_INFO_STREAM(this->get_logger(), "Located goal x: " << roadCentre_.position.x << ", goal y: " << roadCentre_.position.y);
                    }
                    else{setGoal = true;} //reset for next road centre
                }
                //check if there are newly located goals to publish
                if(locatedGoals.poses.size() != 0){
                    goalsPub_->publish(locatedGoals); //publish goals to topic
                    goalsMsg_->poses = locatedGoals.poses; //also assign to global variable to use in timerCallback if goalsCallback fails torecieve message
                    goalProcessed_ = false; //reset boolean value to false
                }
            }
            lck.unlock(); //unlock goals mutex so goalsCallback can process new goals
        }
    }
}

void Mission::sortAndRemoveDuplicates() {
    // sort the vector so that swapped identical pairs are considered duplicates
    std::sort(road_.begin(), road_.end(), [](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& a, const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& b) {
        if ((std::abs(a.first.x - b.first.x) < 1) && (std::abs(a.first.y - b.first.y) < 1)) { //check if first points are equal
            return a.second.x < b.second.x || (std::abs(a.second.x - b.second.x) < 1 && a.second.y < b.second.y); //check if y values are close to each other
        } else {
            return a.first.x < b.first.x || (std::abs(a.first.x - b.first.x) < 1 && a.first.y < b.first.y);
        }
    });

    // remove duplicates, std::unique pushes all duplicates to the end and notifies where unique elements finish
    road_.erase(std::unique(road_.begin(), road_.end(), [](const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& a, const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& b) {
        return (std::abs(a.first.x - b.first.x) < 1 && std::abs(a.first.y - b.first.y) < 1 && std::abs(a.second.x - b.second.x) < 1 && std::abs(a.second.y - b.second.y) < 1) || 
                (std::abs(a.first.x - b.second.x) < 1 && std::abs(a.first.y - b.second.y) < 1 && std::abs(a.second.x - b.first.x) < 1 && std::abs(a.second.y - b.first.y) < 1);
    }), road_.end());
}

geometry_msgs::msg::Point Mission::transformPoint(geometry_msgs::msg::Point point){
    std::unique_lock<std::mutex> lck(odoMtx_); //lock data access to odometry data (pose_)
    geometry_msgs::msg::Point transformedPoint;
    //transfer position and orientation data to tf2 data type format
    tf2::Vector3 audiPoint(pose_.position.x, pose_.position.y, pose_.position.z); //
    tf2::Quaternion q(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
    lck.unlock(); //unlock data access for other threads

    tf2::Transform transform(q,audiPoint); //calculate transform of car pose in global frame
    tf2::Vector3 conePoint(point.x+3.75, point.y, point.z); //create tf2 data type point to be transformed
    tf2::Vector3 transformedVecPoint = transform * conePoint; //apply transform to point
    //assign values of x,y,z to geometry_msgs/msg/Point
    transformedPoint.x = transformedVecPoint.x();
    transformedPoint.y = transformedVecPoint.y();
    transformedPoint.z = transformedVecPoint.z();

    return transformedPoint; //return transformed point
}

void Mission::produceMarkers(geometry_msgs::msg::PoseArray msg, std::string ns, int32_t shape, geometry_msgs::msg::Vector3 size, std_msgs::msg::ColorRGBA colour){
    //creates visualisation markers for input message, with input namespace, shape, size and colour
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto& pose : msg.poses)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world"; //set markers in global frame
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
    markerPub_->publish(marker_array); //pubish visual markers to topic (viewable in simulation)
}

void Mission::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    // RCLCPP_INFO(this->get_logger(), "Mission Odo Callback");
    std::unique_lock<std::mutex> lck(odoMtx_); //lock access to data
    pose_ = msg->pose.pose;
    lck.unlock(); //unlock data access for other threads
    if(!initialOdoTaken_){initialOdoTaken_ = true;} //notify that the initial odometry has been set
}

void Mission::conesCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){ //called when message to cones vector received
    //design visualisation marker (size, shape, colour)
    geometry_msgs::msg::Vector3 size;
    size.x = 0.2; size.y = 0.2; size.z = 0.5;
    std_msgs::msg::ColorRGBA colour;
    colour.a = 1.0; colour.r = 1.0; colour.g = 0.5; colour.b = 0.0;
    //send message under namespace "cones", shape number 3 is CYLINDER
    produceMarkers(*msg,"cones", 3, size, colour);
}

void Mission::timerCallback(){ //function to call goals subscriber if not triggered by published message 
    // RCLCPP_INFO(this->get_logger(), "Timer Callback");
    if(goalsMsg_->poses.size() != 0){
        RCLCPP_INFO(this->get_logger(), "timer calls goals callback");
        goalsCallback(goalsMsg_); //calls goalsCallback function
    }
}

double Mission::distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt){ 
    return sqrt(std::pow(odo.pose.pose.position.x-pt.x,2) + std::pow(odo.pose.pose.position.y-pt.y,2)); //calculate straightline distance to next goal from odometry
}

// void Mission::progress(){
//     // RCLCPP_INFO(this->get_logger(), "Mission Progress Callback");
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
// }

void Mission::run(){ //continuously running thread function
    //initialise variables for progress calculations and distance calculations
    //as well as for validating the goal inbetween the cones
    double dx1, dy1, dx2, dy2;
    std::pair<double, double> euDist;
    float tolerance = 3;
    double estimatedRoadDist = 4;
    bool goalReachable = false;
    double dist;
    nav_msgs::msg::Odometry odo;
    geometry_msgs::msg::Point goal;
    while(rclcpp::ok){// run while node is still active
        // RCLCPP_INFO(this->get_logger(), "Mission Run Callback");
        if(goals_.size() > 0){ //check if there are still goals to be reached
            running_ = true;
        }
        if(startMission_.data && running_){ //if there are goals to be reached AND the mission has been activated by service call or other, start with mission calculations
            //ensure secure data access for odo and goal
            std::unique_lock<std::mutex> lck1(odoMtx_);
            odo = odo_;
            lck1.unlock();
            std::unique_lock<std::mutex> lck2(goalMtx_);
            goal = goals_.front();
            lck2.unlock(); //unlock access to data

            //determine if goal if within track
            for (size_t j=0; j<road_.size(); j++){ //iterate through road pairs
                dx1 = road_.at(j).first.x - goal.x;
                dy1 = road_.at(j).first.y - goal.y;
                dx2 = road_.at(j).second.x - goal.x;
                dy2 = road_.at(j).second.y - goal.y;
                euDist.first = std::hypot(dx1,dy1); //calculate distance to one side
                euDist.second = std::hypot(dx2,dy2); //calculate distance to other side
                if(std::abs(estimatedRoadDist - euDist.first) < tolerance && std::abs(estimatedRoadDist - euDist.second) < tolerance){ 
                    //if goal distance to both sides if roughly 4 (tolerance of 3), goal is within cones and reachable
                    goalReachable = true;
                    break;
                }
            }
            if(goalReachable){ 
                goalReachable = false; //reset goalReachable to false (assume false until proven otherwise)
            }else{
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal cannot be reached, stopping mission."); //send message that goal is unreachable
                startMission_.data = false; //update startMission flag to false and publish to topic to stop the platform
                flagPub_->publish(startMission_);
            }

            dist = distance(odo, goal); //calculate current distance to next goal
            if(dist<tolerance_){ //check if goal has been reached (less than acceptable tolerance)
                goals_.pop_front(); //remove goal from current mission goal vector
                if(goals_.size() > 0){ //check if there are still goals to be travelled to
                    goal = goals_.front(); //reassign goal value
                    initialDistToGoal_ = distance(odo,goal); //calculate new initial distance to goal (for progress)
                    totalMissionDistance_ += initialDistToGoal_; //add initial distance to new goal to total distance variable
                }
                else if(goals_.size() == 0){ //if no more goals to travel to, set running to false
                    running_ = false;
                }
            }
            //update progress percentage
            progress_ = (unsigned int)(100.0*((totalMissionDistance_ - dist)/totalMissionDistance_));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(!rclcpp::ok){
            RCLCPP_INFO_STREAM(this->get_logger(), "Node Killed");
            break;
        }
    }
}

void Mission::detectService(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res){ 
    startMission_.data = req->data; //assigns boolean value of data (from commmand line request) to startMission flag data
    flagPub_->publish(startMission_); //publish flag to topic
    //depending on true or false data value, respond with message accordingly, always sending progress value
    if(req->data){
        res->message = "Mission Running, Progress: " + progress_;
    }
    else{
        res->message = "Stop Mission, Progress: " + progress_;
    }
    if(laserProcessingPtr_->detectConeCentres().size() > 0){ //check if cone is visible from current location
        res->success = true; //respond true if there is a cone
    }
    else{res->success = false;} //respond false otherwise
}