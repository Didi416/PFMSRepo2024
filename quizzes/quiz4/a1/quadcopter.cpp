#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor?

Quadcopter::Quadcopter() :
    TARGET_SPEED(0.4)
{
  tolerance_=0.5;//We set tolerance to be default of 0.5
  type_ = pfms::PlatformType::QUADCOPTER;
};

// We delete the pipes here specifically, which forces them to close before the object is terminated
Quadcopter::~Quadcopter(){
    pfmsConnectorPtr_.reset();
}

bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                              double& distance, double& time,
                              pfms::nav_msgs::Odometry& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;

    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;

    // The estimated goal pose would be the goal, at the angle we had at the origin
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw;
    estimatedGoalPose.linear.x = 0;
    estimatedGoalPose.linear.y = 0;

    return true;
}
bool Quadcopter::calcNewGoal(void) {

    getOdometry();//This will update internal copy of odometry, as well as return value if needed.

    pfms::nav_msgs::Odometry est_final_pos;

    if (!checkOriginToDestination(odo_, goal_.location, goal_.distance, goal_.time, est_final_pos))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x - odo_.position.x;
    double dy = goal_.location.y - odo_.position.y;
    target_angle_ = std::atan2(dy, dx);

    return true;
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {
    // std::cout<<move_f_b<<" : "<<move_l_r<<" : "<<cmd_pipe_seq_<<std::endl;
    cmd_pipe_seq_++;
    pfms::commands::Quadcopter cmd {cmd_pipe_seq_, turn_l_r, move_l_r, move_u_d, move_f_b}; 
    pfmsConnectorPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));//Small delay to ensure message sent
}

bool Quadcopter::reachGoal(void) {
    calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in
    pfms::nav_msgs::Odometry prev_odo = odo_;
    auto start_time = std::chrono::system_clock::now();
    double estimated_time_to_reach_goal = goal_.time;
    pfms::PlatformStatus status = pfms::PlatformStatus::RUNNING;
    pfmsConnectorPtr_->send(status);

    double prevDistance = goal_.distance;
    double prevTime = goal_.time;
    
    //Run below loop until we reach goal
    while (!goalReached()) {

        ///////////////////////////////////////////////////////////////
        //! @todo
        //! TASK 2 - Completion of task requires sending correct values
        //! in sendCmd
        //!
        //! We have access following variables
        //! odo_ - current odometry
        //! target_angle_ - the desired direction of travel
        //! AND the const TARGET_SPEED which is target speed (0.4ms/s)
        //!
        //! We need to compute
        //! dx and dy - the portion of the control in the two axis
        //! in below we just use 0.1 which will not work, it will fly at 45deg

        double dx = TARGET_SPEED*sin(M_PI_2 - target_angle_);
        double dy = TARGET_SPEED*sin(target_angle_);
        

        // Check the syntax of the command, for left/right and forward/backward
        sendCmd(0, dy, 0, dx);

        /////////////////////////////////////////////////////////////////


        // We check if we have not reached goal in close to anticipated time (2s margin)
        // if not reachGoal should be aborted
        double time_since_starting = timeLapsed(start_time);

       // std::cout << "[est/cur] time to goal [" << estimated_time_to_reach_goal << "/" << time_since_starting << "]" << std::endl;


        if(time_since_starting>(estimated_time_to_reach_goal+5.0)){
            // If we have not reached it in the designated time we abandon reaching goal
            // For TASK 3 and 4, consider if anything else needs updating
            return false;
        }

        
        // std::cout<<"Dist: "<<goal_.distance<<"Time: "<<goal_.time<<std::endl;
        
        calcNewGoal(); //get odometry and update target angle of control
        distance_travelled_ += prevDistance - goal_.distance; 
        time_travelled_ += prevTime - goal_.time;
        prevDistance = goal_.distance;
        prevTime = goal_.time;
        ///////////////////////////////////////////////////////////////
        //! @todo
        //! TASK 4 - Update distance travelled
        //!
        //! While the estimated distance at the begining is a rough guide
        //! As the platform moves it could travel more, especially if it
        //! was going against wind (and in case of Ackerman if it was drifting)
        //! Your better of incremeting distance travelled as you go along
        //!
        //! We have access to following variables
        //! odo_ - current position of platform
        //! prev_odo - the previous position
        //! distance_travelled_ - the distance travelled thus far
        
        
        
        //////////////////////////////////////////////////////////////////
    }

    // Stop thq quadcopter immediately
    sendCmd(0, 0, 0, 0);

    calcNewGoal(); //get odometry and update distance to goal
    std::cout<<"Total Dist: "<<distance_travelled_<<"Total Time: "<<time_travelled_<<std::endl;
    ///////////////////////////////////////////////////////////////
    //! @todo
    //! TASK 3 - Update time travelled
    //!
    //! We have access following variables
    //! start_time - the time reaching the goal was started
    //! time_travelled_ - the time we have travelled thus far
    //!
    //! Look at how we compute wether to abort reachGoal
    //! as you could use the same function
    //! Also, consider, if we aborted the function reachGoal, should we
    //! update the time travelled?
    
    //////////////////////////////////////////////////////////////////

    return true;
}


double Quadcopter::timeLapsed(std::chrono::time_point<std::chrono::system_clock> start_time){
    // Update time taken
    auto finish_time = std::chrono::system_clock::now();
    //std::chrono::seconds is integer for some reason, thus duration<double>
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(finish_time - start_time);
    return time_taken.count();
}
