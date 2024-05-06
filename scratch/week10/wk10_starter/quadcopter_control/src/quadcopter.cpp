#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#define DEBUG 1
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG

using std::cout;
using std::endl;
using namespace std::chrono_literals;

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 - Initialisation
//!
//! What do we need to publish to?
//!
//! Publisher example:
//! publisher_member_variable_ = create_publisher<MSG_TPE>("TOPIC",BUFFER_SIZE);  
//!
//!  We also need to declare the publisher_member_variable_ in header 

Quadcopter::Quadcopter() :
    liftoff_(false),TARGET_SPEED(0.4),
    TARGET_HEIGHT_TOLERANCE(0.2)
{
    tolerance_=0.5;//We set tolerance to be default of 0.5


    //! The below is an example of creating a function that is called at a set rate
    //! The function is called every 100ms, this is similar to a thread with an convar waiting on time
    //! The function is called at a set rate, and is not blocking
    timer_ = this->create_wall_timer(
    100ms, std::bind(&Quadcopter::reachGoal, this));

};

Quadcopter::~Quadcopter(){
   
}


bool Quadcopter::checkOriginToDestination(geometry_msgs::msg::Pose origin, 
                            geometry_msgs::msg::Point goal,
                            double& distance, double& time,
                            geometry_msgs::msg::Pose& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;

    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;

    // The estimated goal pose would be the goal, at the angle we had at the origin
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    //estimatedGoalPose.yaw = origin.yaw; - How do we deal with yaw to quaternion?

    return true;
}

bool Quadcopter::calcNewGoal(void) {

    getOdometry();//This will update internal copy of odometry, as well as return value if needed.

    geometry_msgs::msg::Pose est_final_pose;

    if (!checkOriginToDestination(pose_, goal_.location, goal_.distance, goal_.time, est_final_pose))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x - pose_.position.x;
    double dy = goal_.location.y - pose_.position.y;
    target_angle_ = std::atan2(dy, dx);

    return true;
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {

    ///////////////////////////////////////////////////////////////
    //! @todo
    //! TASK 3 - Publishing values
    //!
    //! What do we need to publish here?

    // if(!liftoff_){
    //     pfms::PlatformStatus status = pfms::PlatformStatus::TAKEOFF;    
    //     pipesPtr_->send(status);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));//Small delay to ensure message sent
    //     liftoff_=true;
    // }

    // pfms::commands::UAV cmd = {
    //     cmd_pipe_seq_++,
    //     turn_l_r,
    //     move_l_r,
    //     move_u_d,
    //     move_f_b,
    // };
    // pipesPtr_->send(cmd);

}

bool Quadcopter::reachGoal(void) {
    if(!goalSet_){return false;};

    calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in

    // Get relative target angle
    //double theta = pose_.yaw - target_angle_;
    double theta = tf2::getYaw(pose_.orientation) - target_angle_;


    // Move at `speed` in target direction
    double dx = TARGET_SPEED * std::cos(theta);
    double dy = TARGET_SPEED * std::sin(theta);

    //What about the height?
    double dz=0;

    if(pose_.position.z>(goal_.location.z+TARGET_HEIGHT_TOLERANCE)){
        dz=-0.05;
    }
    if(pose_.position.z<(goal_.location.z+TARGET_HEIGHT_TOLERANCE)){
        dz=+0.05;
    }

    bool reached = goalReached();  

    if(reached){
        goalSet_=false;
        // Stop thq quadcopter immediately
        sendCmd(0, 0, 0, 0);
        ROS_INFO("Goal reached");
    }
    else{
        //Let's send command with these parameters
        sendCmd(0, -dy, dz, dx);
        RCLCPP_INFO_STREAM(get_logger(),"sending: " << dx << " " << -dy << " " << dz);
    }

    return reached;
}
