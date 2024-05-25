#include "mission.h"
#include <iostream>

Mission::Mission(): Node("mission"){
    tolerance_ = 0.5;
    running_ = false;
    initialDistToGoal_ = 0;
    progress_ = 0;
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Mission::goalsCallback,this,std::placeholders::_1));

    markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker",3); 

    progressPub_ = this->create_publisher<std_msgs::msg::String>("orange/progress",3);

    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Mission::odomCallback,this,std::placeholders::_1));  

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Mission::progress, this));

    thread_ = new std::thread(&Mission::run, this);
}

Mission::~Mission(){

}

void Mission::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){

    RCLCPP_INFO(this->get_logger(), "Mission Goals Callback");
    for (auto pose:msg->poses){
        goals_.push_back(pose.position);
    }
    produceMarkers(*msg);

}

void Mission::produceMarkers(geometry_msgs::msg::PoseArray msg){
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto& pose : msg.poses)
    {

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "road";
        marker.id = marker_array.markers.size();
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);

    }
    markerPub_->publish(marker_array);
}

void Mission::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    // RCLCPP_INFO(this->get_logger(), "Mission Odo Callback");
    std::unique_lock<std::mutex> lck(odoMtx_);
    odo_ = *msg;
    lck.unlock();  
}

double Mission::distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt){ 
    return sqrt(std::pow(odo.pose.pose.position.x-pt.x,2) + std::pow(odo.pose.pose.position.y-pt.y,2));
}

void Mission::progress(){
    // RCLCPP_INFO(this->get_logger(), "Mission Progress Callback");
    std_msgs::msg::String progressMsg = std_msgs::msg::String();
    if(running_){
        nav_msgs::msg::Odometry odo;
        geometry_msgs::msg::Point goal;

        std::unique_lock<std::mutex> lck1(odoMtx_);
        odo = odo_;
        lck1.unlock();
            
        std::unique_lock<std::mutex> lck2(goalsMtx_);
        goal = goals_.front();
        lck2.unlock();

        double dist = distance(odo,goal);

        progress_ = (unsigned int)(100.0*((initialDistToGoal_ - dist)/initialDistToGoal_));
    }
    // RCLCPP_INFO(this->get_logger(), "Mission Progress: " + progress_);
    progressMsg.data = progress_;
    progressPub_->publish(progressMsg);
}

void Mission::run(){
    while(rclcpp::ok){
        // RCLCPP_INFO(this->get_logger(), "Mission Run Callback");
        nav_msgs::msg::Odometry odo;
        geometry_msgs::msg::Point goal;

        std::unique_lock<std::mutex> lck1(odoMtx_);
        odo = odo_;
        lck1.unlock();
        
        std::unique_lock<std::mutex> lck2(goalsMtx_);
        goal = goals_.front();
        lck2.unlock();

        if(running_){
            double dist = distance(odo, goal);
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
        }

        else{
            if(goals_.size() > 0){
                running_ = true;
            }
        }
    }
}