#include "skidsteer.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

SkidSteer::SkidSteer(){
    platformType_ = pfms::PlatformType::SKIDSTEER;
    velocity_ = 1.0; //m/s
    angularV_ = 1.0; //rad/s
    i_ = 1;
    turnLR_ = 0.0;
    moveFB_ = 0.0;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>();
}

bool SkidSteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    //Skidsteer can reach all goals, as platform turns in place then moved forwards to goal.
    angularDisp_ = atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x)) - currentOdo_.yaw;

    distance = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
    
    time = distance/velocity_;
    std::cout<<"Time to go is: "<<time<<std::endl;
    if (angularDisp_ != 0){
        time += std::abs(angularDisp_);
        std::cout<<"Time with turning: "<<time<<std::endl;
    }
    return true;
}

double SkidSteer::distanceToGoal(void){    
    return distanceToCurrentGoal_;
}

double SkidSteer::timeToGoal(void){
    return timetoCurrentGoal_;
}

void SkidSteer::drive(unsigned long i, double turnLR, double moveFB){
    pfms::commands::SkidSteer cmd {i,turnLR,moveFB}; 

    pfmsConnectorPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    pfmsConnectorPtr_->read(currentOdo_,platformType_);
}

bool SkidSteer::reachGoal(void){
    bool goalReached = false;

    turningAngle_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
    if (turningAngle_ < 0){
        angularV_ = 1.0;
    }
    else if (turningAngle_ > 0){
        angularV_ = -1.0;
    }
    velocity_ = 1.0;
    
    while (!goalReached){
        std::cout<<"Angular Disp: "<<angularDisp_<<std::endl;
        std::cout<<"Angular Velo: "<<angularV_<<std::endl;
        if (angularDisp_ > 0){
            if (angularDisp_ >= 0.05){
                drive(1, angularV_, 0);
                std::cout<<"TURNING"<<std::endl;
                angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                if (angularDisp_ < 0.3){
                    angularV_ = angularV_*0.7;
                }
            }
            if (angularDisp_ <= 0.05 && distanceToGoal() >= 0.4){
                angularV_ = 0.5;
                drive(1, 0, velocity_);
                std::cout<<"DRIVING: Distance to Goal: "<<distanceToCurrentGoal_<<std::endl;
                distanceToCurrentGoal_ = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
                // angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                if (distanceToGoal() < 1){
                    velocity_ = 0.1;
                }
            }
            if (angularDisp_ <= 0.05 && distanceToGoal() <= 0.4){
                std::cout<<"GOAL REACHED"<<std::endl;
                velocity_ = 0;
                angularV_ = 0;
                return true;
            }
        }

        else if (angularDisp_ < 0){
            if (angularDisp_ <= -0.05){
                drive(1, angularV_, 0);
                std::cout<<"TURNING2"<<std::endl;
                angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                if (angularDisp_ > 0.3){
                    angularV_ = angularV_*0.75;
                }
            }
            if (angularDisp_ >= -0.05 && distanceToGoal() >= 0.4){
                drive(1, 0, velocity_);
                std::cout<<"DRIVING: Distance to Goal: "<<distanceToCurrentGoal_<<std::endl;
                distanceToCurrentGoal_ = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
                // angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                if (distanceToGoal() < 1){
                    velocity_ = 0.1;
                }
            }
            if (angularDisp_ >= -0.05 && distanceToGoal() <= 0.4){
                std::cout<<"GOAL REACHED"<<std::endl;
                return true;
            }
        }
    }
    return false;
}

