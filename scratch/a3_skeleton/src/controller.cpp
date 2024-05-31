#include "controller.h"

Controller::Controller(): Node("controller") //Node name is "a3_skeleton"
{
    startMission_ = false; //initialise as false, to sit idle until service call to activate
    //create subscriptions to odometry, goals and flag_mission topics, binding to callback functions
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 10, 
                                std::bind(&Controller::odomCallback,this,std::placeholders::_1)); //audi's position and orientation inglobal frame
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 10, 
                                std::bind(&Controller::goalsCallback,this,std::placeholders::_1)); //goals to be travelled to either from a3_support or mission node
    flagSub_ = this->create_subscription<std_msgs::msg::Bool>("orange/flag_mission", 10, 
                                std::bind(&Controller::flagCallback,this,std::placeholders::_1)); //flag to start/stop mission as dictated in mission node
}

Controller::~Controller(){ //destructor
    if(thread_->joinable()){
        thread_->join(); //join threads upon destruction of node
    }
}

void Controller::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    // RCLCPP_INFO(this->get_logger(), "Odo Callback");
    std::unique_lock<std::mutex> lck(odoMtx_); //locks mutex for secure data access
    pose_ = msg->pose.pose; //assigns value to pose_ (updates global odometry values)
    lck.unlock(); //unlocks mutex to allow other threads access to data
}

void Controller::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg){
    RCLCPP_INFO(this->get_logger(), "Goals Callback"); //signals subscription successful
    setGoals(*msg); //set goals/add goals to controller mission
    
}

void Controller::setGoals(geometry_msgs::msg::PoseArray msg){
    for (size_t i=0; i<msg.poses.size(); i++){ //iterate through message array
        pfmsGoals_.push_back({msg.poses.at(i).position.x, msg.poses.at(i).position.y, msg.poses.at(i).position.z}); //add new goals to end of goals vector (deque)
    }
}

void Controller::flagCallback(const std::shared_ptr<std_msgs::msg::Bool> msg){
    startMission_ = msg->data; //upon recieving message, assign the value to startMission, which will either start or stop the mission at this point in time
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){ //store odometry in pfms data type to be used in audi library calculations
    std::unique_lock<std::mutex> lck(odoMtx_); //lock odometry data access
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current Odo: " << pose_.position.x << ", " << pose_.position.y);

    //copy x, y and z values of pose_ to currentOdo_ (double data types are transferrable)
    currentOdo_.position.x = pose_.position.x;
    currentOdo_.position.y = pose_.position.y;
    currentOdo_.position.z = pose_.position.z;
    currentOdo_.yaw = tf2::getYaw(pose_.orientation); //convert quaternion (orientation) to yaw angle
    lck.unlock(); //unlock lutex and allow other threads to access data
    return currentOdo_;
}