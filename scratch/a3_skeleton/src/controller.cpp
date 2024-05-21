#include "controller.h"

Controller::Controller()
    : Node("a3_skeleton") //Node name is "a3_skeleton"
{
    //Subscribing to laser, this will call the laserCallback function when a new message is published
    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("orange/laserscan", 10, 
                                std::bind(&Controller::laserCallback,this,std::placeholders::_1));   

    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/laserscan", 10, 
                                std::bind(&Controller::odomCallback,this,std::placeholders::_1));   

    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/laserscan", 10, 
                                std::bind(&Controller::goalsCallback,this,std::placeholders::_1));    

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
}

void Controller::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    std::unique_lock<std::mutex> lck(poseMtx_);
    pose_ = msg->pose.pose;
}

geometry_msgs::msg::Pose Controller::getOdometry(void){
    std::unique_lock<std::mutex> lck(odoMtx_);
    geometry_msgs::msg::Pose pose=pose_;
    return pose;
}