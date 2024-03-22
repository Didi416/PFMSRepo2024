#include "ackerman.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include "audi.h"

Ackerman::Ackerman(){
    platformType_ = pfms::PlatformType::ACKERMAN;
    STEERING_RATIO = 17.3;
    LOCK_TO_LOCK_REVS = 3.2;
    MAX_STEER_ANGLE = (M_PI * LOCK_TO_LOCK_REVS / STEERING_RATIO); //radians
    WHEELBASE = 2.65; //m
    MAX_BRAKE_TORQUE = 8000.0; //Nm
    DEFAULT_THROTTLE = 0.1; //m/s
    i_ = 1;
    brake_ = 0.0;
    steering_ = 0.0;
    throttle_ = 0.0;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>();
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    Audi audi;
    if (audi.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose)){
        distanceToCurrentGoal_ = distance;
        timetoCurrentGoal_ = time;
        return true;
    }
    else {
        return false;
    }
}

double Ackerman::distanceToGoal(void){    
    return distanceToCurrentGoal_;
}

double Ackerman::timeToGoal(void){
    return timetoCurrentGoal_;
}

bool Ackerman::reachGoal(void){
    //Initialise conditions for new reach goal command
    unsigned long repeats = 1;
    throttle_ = 0.1;
    int state = 1;
    //compute steering using Audi library and store the value in steering_ (private data member)
    double distance;
    Audi audi;
    audi.computeSteering(currentOdo_, goal_, steering_, distance);
    //Display goal point in gazebo sim (rviz)
    unsigned int j=0;
    pfms::geometry_msgs::Goal goal{j++,goal_};
    pfmsConnectorPtr_->send(goal);
    while (true){
        pfmsConnectorPtr_->read(currentOdo_,platformType_);

        pfms::commands::Ackerman cmd {repeats,brake_,steering_,throttle_};
        std::cout<<"Brake: "<<brake_<<" Steering: "<<steering_<<" Throttle: "<<throttle_<<std::endl;

        pfmsConnectorPtr_->send(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        pfmsConnectorPtr_->read(currentOdo_,platformType_);
        distanceToCurrentGoal_ = sqrt(pow(currentOdo_.position.x - goal_.x, 2) + pow(currentOdo_.position.y - goal_.y, 2));
        std::cout<<"Distance to Goal: "<<distanceToCurrentGoal_<<std::endl;
        
        switch (state){
            case 0:
                std::cout<<"Finished"<<std::endl;
                brake_ = 0.0;
                steering_ = 0.0;
                throttle_ = 0.0;
                return true;
            case 1: //start driving, until distance to goal is less than 1m, then switch to apply brakes
                repeats++;
                if (distanceToCurrentGoal_ < 1.0){
                    state = 2;
                }
                break;
            case 2: //apply brakes until stopped at goal
                throttle_ = 0;
                brake_ = 3000;
                if (distanceToCurrentGoal_ < 0.5){
                    state = 0;
                }
                break;
        }
        
        if (distanceToCurrentGoal_ >= 0.5){
            repeats++;
        }

        
    }
}

pfms::nav_msgs::Odometry Ackerman::getOdometry(void){
    pfmsConnectorPtr_->read(currentOdo_,platformType_);

    return currentOdo_;
}
