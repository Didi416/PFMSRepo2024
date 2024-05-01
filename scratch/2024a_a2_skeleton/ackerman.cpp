#include "ackerman.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include <thread>
#include "audi.h"

Ackerman::Ackerman(){
    platformType_ = pfms::PlatformType::ACKERMAN;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>();
    MAX_BRAKE_TORQUE = 8000.0; //Nm
    i_ = 1;
    brake_ = 0.0;
    steering_ = 0.0;
    throttle_ = 0.0;
    velocity_ = 2.91;
    goalTolerance_ = 0.5; //default goal tolerance for when unit tests do not set it.
    distanceTravelled_ = 0.0;
    timeTravelled_ = 0.0;
}

Ackerman::~Ackerman(){

}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
    //uses Audi library which 'automatically' computes values for distance, time and etimated goal pose and if the goal can be reached by Ackerman.
    Audi audi;
    if (audi.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose)){
        return true;
    }
    else {
        return false;
    }
    
}

void Ackerman::run(void){
    // platformStatus_ = pfms::PlatformStatus::RUNNING;
    // std::thread ackermanRun(&Ackerman::reachGoals, this);
    // ackermanRun.detach();
  
    running_ = true;
    std::unique_lock<std::mutex> lck(mtxStart_);
    mtxStart_.unlock();
    cvStart_.notify_all();
    pfmsConnectorPtr_->send(platformStatus_);
}

void Ackerman::reachGoals(void){
    std::unique_lock<std::mutex> lck(mtxStart_);
    cvStart_.wait(lck, [&](){return running_==true;});
    while(running_){
        bool goalReached;
        unsigned long repeats;
        int state;
        double prevDistance;
        pfmsConnectorPtr_->send(platformStatus_);
        pfms::commands::Ackerman cmd;
        for (int i=0; i<goals_.size(); i++){
            unsigned int j=0;
            pfms::geometry_msgs::Goal goalAck{j++,goals_.at(i)};
            pfmsConnectorPtr_->send(goalAck);
        }
        for (int i=0; i<goals_.size(); i++){
            goalReached = false;
            //Initialise (or reset) conditions for new reach goal command
            repeats = 1;
            throttle_ = 0.2;
            state = 1;
            //compute steering using Audi library and store the value in steering_ (private data member)
            Audi audi;
            audi.computeSteering(currentOdo_, goals_.at(i), steering_, prevDistance);
            checkOriginToDestination(currentOdo_,goals_.at(i),distanceToCurrentGoal_,timetoCurrentGoal_,estimatedGoalPose_);
            //Display goal point in gazebo sim (rviz)
            while (!goalReached){
                //takes in brake, steering and throttle values and inputs to Ackerman cmd, sending commands through pfmsConnector and reads current odo
                getOdometry();
                cmd = {repeats,brake_,steering_,throttle_};
                pfmsConnectorPtr_->send(cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); //wait for a period to allow information to be sent then retrieved
                checkOriginToDestination(currentOdo_,goals_.at(i),distanceToCurrentGoal_,timetoCurrentGoal_,estimatedGoalPose_);
                pfmsConnectorPtr_->read(currentOdo_,platformType_);
                switch (state){
                    case 0:
                        brake_ = 0.0;
                        steering_ = 0.0;
                        throttle_ = 0.0;
                        goalReached = true;
                    case 1: //start driving, until distance to goal is less than 1m, then switch to apply brakes
                        if (distanceToGoal() < 2){
                            state = 2;
                        }
                        break;
                    case 2: //apply brakes until close at goal
                        throttle_ = 0;
                        brake_ = 5800; //less than max braking, to just gradually slow down
                        if (distanceToGoal() < goalTolerance_){
                            state = 3;
                        }
                        break;
                    case 3: //apply max braking torque to come to a complete stop
                        brake_ = MAX_BRAKE_TORQUE;
                        if(currentOdo_.linear.x <= 0 && currentOdo_.linear.y <= 0){ //check for when velocity is 0 (stopped) before proceeding with next goal or terminating (finishing) program
                            state = 0;
                        }
                        break;
                }
                velocity_ = sqrt(pow(currentOdo_.linear.x, 2)+pow(currentOdo_.linear.y, 2));
                updateDistTime(velocity_);
                
                if (distanceToGoal() <= goalTolerance_){ //incremental counter for sending commands (needs increasing sequence counter)
                    if (i == goals_.size()-1){
                        platformStatus_ = pfms::PlatformStatus::IDLE;   
                    }
                }else{repeats++;}        
            }
        }
        platformStatus_ = pfms::PlatformStatus::IDLE;
        pfmsConnectorPtr_->send(platformStatus_);
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

// bool Quadcopter::goQuad() {
//     std::unique_lock<std::mutex> lck(mtxStart_);
//     cvStart_.wait(lck, [&](){return running_==true;});
//     while(running_ == true){
//     	//Wrap all of the code you had in the go function including variable declaration such as "double x  = 0;
//     	//including the while loop up until the return


//     // Need a way to exit this loop, the code below will exit the loop when the thread logic
//     // is out of scope
//      if(!running_){
//       break;
//     }
//       std::this_thread::sleep_for (std::chrono::milliseconds(10));
//       break;
//     }
//    return true;
// }