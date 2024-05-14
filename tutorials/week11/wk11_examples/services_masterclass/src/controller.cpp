#include "controller.h"
#include <cmath>

using std::placeholders::_1;

/**
 * \brief Shared functionality/base class for platform controllers
 *
 */
Controller::Controller() :
    Node("controller"),
    goalSet_(false),    
    distance_travelled_(0),
    time_travelled_(0),
    cmd_pipe_seq_(0),
    status_(pfms::PlatformStatus::IDLE)
{
    // We open up the pipes here in the constructor, so we can OPEN them once ONLY
    //pipesPtr_ = new Pipes();
    // Now we create a node handle in derived class (as they have custom messages/topics)  
    // We still have one message we could potentialy use (odo)
    sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/gt_odom", 1000, std::bind(&Controller::odoCallback,this,_1));
    sub2_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 1000, std::bind(&Controller::setGoal,this,_1));

    srv1_ = this->create_service<std_srvs::srv::SetBool>(
        "/reach_goal", std::bind(&Controller::control,this,std::placeholders::_1, std::placeholders::_2));      

    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
};

//We would now have to sacrifice having a return value to have a setGoal
//At week 10 we do not know about services (which allow us to retrun value
//So to allow to set a goal via topic we forfit having areturn value for now
//At week 11 you can replace this with a service
//bool Controller::setGoal(geometry_msgs::Point goal) {
void Controller::setGoal(const geometry_msgs::msg::Point& msg){    
  goal_.location = msg;
  goalSet_=true;
  //return calcNewGoal();
}

bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;
}
double Controller::timeToGoal(void) {
    return goal_.time;
}
double Controller::distanceTravelled(void) {
    return distance_travelled_;
}
double Controller::timeInMotion(void) {
    return time_travelled_;
}

bool Controller::goalReached() {
    double dx = goal_.location.x - pose_.position.x;
    double dy = goal_.location.y - pose_.position.y;
    double dz = goal_.location.z - pose_.position.z;

    return (pow(pow(dx,2)+pow(dy,2)+pow(dz,2),0.5) < tolerance_);
}


void Controller::odoCallback(const nav_msgs::msg::Odometry& msg){
    std::unique_lock<std::mutex> lock(poseMtx_);
    pose_ = msg.pose.pose;
}

//Do we need below ... maybe we can use it and impose a mutex
//To secure data?
geometry_msgs::msg::Pose Controller::getOdometry(void){
    std::unique_lock<std::mutex> lock(poseMtx_);
    return pose_;
}

std::string Controller::getInfoString()
{
    std::stringstream ss;
    switch(status_)
    {
        case pfms::PlatformStatus::IDLE   : ss << "IDLE ";    break;
        case pfms::PlatformStatus::RUNNING : ss << "RUNNING ";  break;
        case pfms::PlatformStatus::TAKEOFF : ss << "TAKEOFF ";  break;
        case pfms::PlatformStatus::LANDING : ss << "LANDING ";  break;
    }
    return ss.str(); // This command convertes trsingstream to string
}