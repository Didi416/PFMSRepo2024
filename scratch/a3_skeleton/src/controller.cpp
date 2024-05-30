#include "controller.h"

Controller::Controller(): Node("controller") //Node name is "a3_skeleton"
{
    startMission_ = false;
    
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Controller::odomCallback,this,std::placeholders::_1));   
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Controller::goalsCallback,this,std::placeholders::_1));
    flagSub_ = this->create_subscription<std_msgs::msg::Bool>("orange/flag_mission", 10, 
                                std::bind(&Controller::flagCallback,this,std::placeholders::_1));    
}

Controller::~Controller(){
    if(thread_->joinable()){
        thread_->join();
    }
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
    std::cout<<"goal size: "<<pfmsGoals_.size()<<std::endl;
    
}

void Controller::setGoals(geometry_msgs::msg::PoseArray msg){
    for (size_t i=0; i<msg.poses.size(); i++){
        pfmsGoals_.push_back({msg.poses.at(i).position.x, msg.poses.at(i).position.y, msg.poses.at(i).position.z});
    }
}

void Controller::flagCallback(const std::shared_ptr<std_msgs::msg::Bool> msg){
    startMission_ = msg->data;
}

// void Controller::timerCallback(){
//     // RCLCPP_INFO(this->get_logger(), "Timer Callback");
//     if(goalsMsg_->poses.size() != 0){
//         RCLCPP_INFO(this->get_logger(), "timer calls goals callback");
//         goalsCallback(goalsMsg_);
//     }
// }

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