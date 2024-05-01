#include "quadcopter.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Quadcopter::Quadcopter(){
    platformType_ = pfms::PlatformType::QUADCOPTER;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>();
    velocity_ = 1.0;
    turnLR_ = 0.0;
    moveLR_ = 0.0;
    moveUD_ = 0.0;
    moveFB_ = 0.0;
    distanceTravelled_ = 0.0;
    timeTravelled_ = 0.0;
}
Quadcopter::~Quadcopter(){
    
}

bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    distance = sqrt(pow(origin.position.x - goal.x, 2) + pow(origin.position.y - goal.y, 2));
    time = distance/velocity_;
    estimatedGoalPose.position = goal;
    return true; //quadcopter can reach any and all goals
}

void Quadcopter::run(void){
    platformStatus_ = pfms::PlatformStatus::TAKEOFF;
    std::thread quadcopterRun(&Quadcopter::reachGoals, this);
    quadcopterRun.detach();
}

void Quadcopter::fly(unsigned long i, double turnLR, double moveLR, double moveUD, double moveFB){
    //takes in turn and move values (angular velocity and velocity) and inputs to SkidSteer cmd, sending commands through pfmsConnector and reads current odo
    pfms::commands::Quadcopter cmd {i,turnLR,moveLR,moveUD,moveFB}; 
    pfmsConnectorPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); //wait for a period to allow information to be sent then retrieved
    pfmsConnectorPtr_->read(currentOdo_,platformType_);
}

bool Quadcopter::navCalcs(pfms::geometry_msgs::Point goal){
    if (!checkOriginToDestination(getOdometry(), goal, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal.x - currentOdo_.position.x;
    double dy = goal.y - currentOdo_.position.y;
    target_angle_ = std::atan2(dy, dx);
    return true;
}

void Quadcopter::reachGoals(void){
    pfmsConnectorPtr_->send(platformStatus_);
    double prevDistance = 0.0;;
    unsigned long repeats;
    bool goalReached;
    double dx, dy, dz, vx, vy, vz;
    for (int i=0; i<goals_.size(); i++){
        navCalcs(goals_.at(i));
        prevDistance = distanceToGoal();
        goalReached = false;
        //Initialise (or reset) conditions for new reach goal command
        repeats = 1;
        //Display goal point in gazebo sim (rviz)
        unsigned int j=0;
        pfms::geometry_msgs::Goal goal{j++,goals_.at(i)};
        pfmsConnectorPtr_->send(goal);
        while (!goalReached){
            fly(repeats,turnLR_,moveLR_,moveUD_,moveFB_);
            navCalcs(goals_.at(i));
            vx = velocity_*cos(target_angle_);
            vy = velocity_*sin(target_angle_);
            vz = velocity_;
            // std::cout<<"Target Angle: "<<target_angle_<<std::endl;
            dx = goals_.at(i).x - currentOdo_.position.x;
            dy = goals_.at(i).y - currentOdo_.position.y;
            dz = 1 - currentOdo_.position.z;
            switch(platformStatus_){
                case pfms::PlatformStatus::IDLE:
                    turnLR_ = 0.0;
                    moveLR_ = 0.0;
                    moveUD_ = 0.0;
                    moveFB_ = 0.0;
                    break;
                case pfms::PlatformStatus::TAKEOFF:
                    // std::cout<<"Quad TAKEOFF";
                    moveUD_ = vz;
                    if (std::abs(dz) < 0.2){
                        // std::cout<<"Height of 2m reached"<<std::endl;
                        moveUD_ = 0;
                        platformStatus_ = pfms::PlatformStatus::RUNNING;
                    }
                    break;
                case pfms::PlatformStatus::RUNNING:
                    // std::cout<<"Quad RUNNING";
                    moveFB_ = vx;
                    moveLR_ = vy;
                    if (std::abs(dx) < 0.25){
                        moveFB_ = 0;
                    }
                    if (std::abs(dy) < 0.25){
                        moveLR_ = 0;
                    }
                    break;
                case pfms::PlatformStatus::LANDING:
                    break;
            }
            updateDistTime(velocity_);

            if (distanceToGoal() <= goalTolerance_){ //incremental counter for sending commands (needs increasing sequence counter)
                goalReached = true;
                if (i == goals_.size()-1){
                    platformStatus_ = pfms::PlatformStatus::IDLE;   
                }
            }else{repeats++;}
        }
    }
}