#include "ackerman.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

Ackerman::Ackerman(){
    platformType_ = pfms::PlatformType::ACKERMAN;
    STEERING_RATIO = 17.3;
    LOCK_TO_LOCK_REVS = 3.2;
    MAX_STEER_ANGLE = (M_PI * LOCK_TO_LOCK_REVS / STEERING_RATIO); //radians
    WHEELBASE = 2.65; //m
    MAX_BRAKE_TORQUE = 8000.0; //Nm
    DEFAULT_THROTTLE = 2.91; //m/s
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    double oldX;
    double oldY;
    double r;
    double angleD;
    double timeStep = 0.05;
    double currentVelocity = DEFAULT_THROTTLE;
    pfms::nav_msgs::Odometry originPos = origin;
    double originalDistance = hypot(originPos.position.x - goal.x, originPos.position.y - goal.y);
    std::cout<<"OG x pos: "<<originPos.position.x<<std::endl;
    std::cout<<"Stored x pos: "<<currentOdo_.position.x<<std::endl;
    std::cout<<"OG y pos: "<<originPos.position.y<<std::endl;
    std::cout<<"Stored y pos: "<<currentOdo_.position.y<<std::endl;
    while (true){
        double angleToGoal = atan2(goal.y - originPos.position.y, goal.x - originPos.position.x);
        // double steeringAngle = std::min(MAX_STEER_ANGLE, std::max(-MAX_STEER_ANGLE, angleToGoal - originPos.yaw));
        // std::cout<<"Odo readings"<<std::endl;
        // getOdometry();
        
        //Updates position and heading using differential drive kinematics
        oldX = originPos.position.x;
        oldY = originPos.position.y;
        originPos.yaw += (currentVelocity/WHEELBASE)*tan(angleToGoal) * timeStep;
        originPos.position.x += currentVelocity * cos(originPos.yaw) * timeStep;
        originPos.position.y += currentVelocity * sin(originPos.yaw) * timeStep;
        std::cout<<hypot(originPos.position.x - goal.x, originPos.position.y - goal.y)<<std::endl;

        r = sqrt(pow(originPos.position.x - oldX, 2) + pow (originPos.position.y - oldY, 2));
        angleD = atan2(originPos.position.y - oldY, originPos.position.x - oldX);
        distance += r * std::abs(angleD);
        std::cout<<"Distance is: "<<distance<<std::endl;

        if (hypot(originPos.position.x - goal.x, originPos.position.y - goal.y)<0.1 || std::abs(angleToGoal - originPos.yaw)<0.1){
            return true; //return true if goal is reached or current trajectory (angle) is headed for the goal.
        }
        else if (hypot(originPos.position.x - goal.x, originPos.position.y - goal.y) > originalDistance){
            return false; //returns false if trajectory is moving away from goal, goal cannot be reached
        }
    }
}

double Ackerman::distanceToGoal(void){
    //distance = radius * theta (assume half circle so 180 deg/pi rad)
    double distanceToGo = 0;
    double radius = 0;
    double theta = M_PI;
    
    return distanceToGo;
}

double Ackerman::timeToGoal(void){
    double timeLeft = 0;
    //using time = distance to go divided by current speed (iterating for loop as both values consistently change)
    return timeLeft;
}

bool Ackerman::reachGoal(void){
    
    return true;
}

pfms::nav_msgs::Odometry Ackerman::getOdometry(void){
    pfms::nav_msgs::Odometry odo;
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>();
    pfmsConnectorPtr->read(odo,getPlatformType());

    pfms::commands::Ackerman cmd {
        i_,
        brake_,
        steering_,
        throttle_,
    };

    pfmsConnectorPtr->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    bool OK  =  pfmsConnectorPtr->read(odo,platformType_);

    if(OK){
        std::cout << 
            i_ << " " <<
            odo.time << " " <<
            odo.position.x << " " <<
            odo.position.y << " " <<
            odo.yaw << " " <<
            odo.linear.x << " " <<
            odo.linear.y << std::endl;
    }

    currentOdo_ = odo;

    return odo;
}
