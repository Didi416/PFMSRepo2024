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
}

bool SkidSteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    //Skidsteer can reach all goals, as platform turns in place then moved forwards to goal.
    angularDisp_ = (atan2((currentOdo_.position.y - goal_.y), (currentOdo_.position.x - goal_.x))) - (currentOdo_.yaw);
    turningAngle_ = std::abs(angularDisp_);
    
    if (angularDisp_ < 0){
        angularV_ = 1.0;
    }
    else if (angularDisp_ > 0){
        angularV_ = -1.0;
    }

    // std::cout<<"Current Odo Readings: "<<std::endl;
    // std::cout<<currentOdo_.position.x<<std::endl;
    // std::cout<<currentOdo_.position.y<<std::endl;
    // std::cout<<currentOdo_.yaw<<std::endl;

    return true;
}

double SkidSteer::distanceToGoal(void){
    double distanceToGo = 0;
    distanceToGo = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
    std::cout<<"Distance is: "<<distanceToGo<<std::endl;
    return distanceToGo;
}

double SkidSteer::timeToGoal(void){
    double timeLeft = 0;
    timeLeft = distanceToGoal()/velocity_;
    std::cout<<"Time to go is: "<<timeLeft<<std::endl;
    if (turningAngle_ != 0){
        timeLeft += (turningAngle_/angularV_);
        std::cout<<"Time added: "<<timeLeft<<std::endl;
    }
    std::cout<<"Final Time: "<<timeLeft<<std::endl;
    return timeLeft;
}

void SkidSteer::drive(unsigned long i, double turnLR, double moveFB, std::shared_ptr<PfmsConnector> pfmsConnectorPtr){
    pfms::nav_msgs::Odometry odo;
    pfms::commands::SkidSteer cmd {i,turnLR,moveFB}; 

    pfmsConnectorPtr->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    bool OK  =  pfmsConnectorPtr->read(odo,platformType_);
    currentOdo_ = odo;
}

bool SkidSteer::reachGoal(void){
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>();
    bool goalReached = false;

    velocity_ = 1.0;
    double originalAngleDisp = angularDisp_;
    
    while (!goalReached){
    // std::cout<<"Distance to Goal: "<<std::endl;
    // std::cout<<distanceToGoal()<<std::endl;
    // std::cout<<currentOdo_.position.x<<std::endl;
    // std::cout<<currentOdo_.position.y<<std::endl;
    // std::cout<<currentOdo_.yaw<<std::endl;
        std::cout<<"Angular Disp: "<<angularDisp_<<std::endl;
        if (angularDisp_ > 0){
            if (angularDisp_ >= 0.08){
                drive(1, angularV_, 0, pfmsConnectorPtr);
                std::cout<<"TURNING"<<std::endl;
                angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                if (angularDisp_ < 0.3){
                    angularV_ = angularV_*0.75;
                }
            }
            if (angularDisp_ <= 0.08 && distanceToGoal() >= 0.4){
                drive(1, 0, velocity_, pfmsConnectorPtr);
                std::cout<<"DRIVING"<<std::endl;
                if (distanceToGoal() < 1){
                    velocity_ = 0.1;
                }
            }
            if (angularDisp_ <= 0.08 && distanceToGoal() <= 0.4){
                std::cout<<"GOAL REACHED"<<std::endl;
                return true;
            }
        }

        else if (angularDisp_ < 0){
            if (angularDisp_ <= -0.08){
                drive(1, angularV_, 0, pfmsConnectorPtr);
                std::cout<<"TURNING2"<<std::endl;
                angularDisp_ = (atan2(std::abs(currentOdo_.position.y - goal_.y), std::abs(currentOdo_.position.x - goal_.x))) - std::abs(currentOdo_.yaw);
                if (angularDisp_ > 0.3){
                    angularV_ = angularV_*0.75;
                }
            }
            if (angularDisp_ >= -0.08 && distanceToGoal() >= 0.4){
                drive(1, 0, velocity_, pfmsConnectorPtr);
                std::cout<<"DRIVING"<<std::endl;
                if (distanceToGoal() < 1){
                    velocity_ = 0.1;
                }
            }
            if (angularDisp_ >= -0.08 && distanceToGoal() <= 0.4){
                std::cout<<"GOAL REACHED"<<std::endl;
                return true;
            }
        }
    }
    return false;
}

pfms::nav_msgs::Odometry SkidSteer::getOdometry(){
    pfms::nav_msgs::Odometry odo;
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>();

    pfms::commands::SkidSteer cmd {i_,turnLR_,moveFB_}; 

    pfmsConnectorPtr->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    bool OK  =  pfmsConnectorPtr->read(odo,platformType_);

    currentOdo_ = odo;

    return odo;
}
