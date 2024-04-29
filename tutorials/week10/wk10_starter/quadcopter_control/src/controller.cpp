#include "controller.h"
#include <cmath>

using std::placeholders::_1;

/**
 * \brief Shared functionality/base class for platform controllers
 *
 */

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 
//!
//! What do we need to subscribe to?
//!
//! We need to create the subscriber in the constructor
//! We need a callback function that we need to declare in the header as well and the <MSG_TYPE> needs to be in subscriber.
//! We need to create the callback member varaible as well in the header.

Controller::Controller() :
    Node("controller"),
    goalSet_(false),    
    distance_travelled_(0),
    time_travelled_(0),
    cmd_pipe_seq_(0)
{
    // In A1/A2 we open up the pipes here in the constructor, so we can OPEN them once ONLY
    // pipesPtr_ = new Pipes();
    // Now we create a node subscriber/publisher in base of derived class (as they have custom messages/topics)  
    //
    // Recall syntax for subscribers
    // sub_member_variable_ = create_subscription<MSG_TPE>("TOPIC",BUFFER_SIZE,std::bind(&CLASS::CALLBACK,this,_1));
    // 
    // HINT: We need to know where the platform is!
    

    // We have a subscriber for goals to be send directly
    sub2_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 5, std::bind(&Controller::setGoal,this,_1));

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

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 
//!
//! What do we do here?
// void Controller::odoCallback

//Do we need below ... maybe we can use it and impose a mutex
//To secure data?
geometry_msgs::msg::Pose Controller::getOdometry(void){
    return pose_;
}

// pfms::PlatformType Controller::getPlatformType(void){
//     return type_;
// }
