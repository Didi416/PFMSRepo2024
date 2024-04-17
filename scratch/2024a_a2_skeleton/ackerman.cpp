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
    DEFAULT_THROTTLE = 0.1; //m/s
    i_ = 1;
    brake_ = 0.0;
    steering_ = 0.0;
    throttle_ = 0.0;
    goalTolerance_ = 0.5; //default goal tolerance for when unit tests do not set it.
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
    platformStatus_ = pfms::PlatformStatus::RUNNING;
    std::thread ackermanRun(&Ackerman::reachGoals, this);
    ackermanRun.detach();
}

void Ackerman::reachGoals(void){
    bool goalReached;
    unsigned long repeats;
    int state;
    double originalDistance = distanceToGoal();
    double straightDistToCurrentGoal;
    pfmsConnectorPtr_->send(platformStatus_);
    for (int i=0; i<goals_.size(); i++){
        currentGoal_ = goals_.at(i);
        goalReached = false;
        //Initialise (or reset) conditions for new reach goal command
        repeats = 1;
        throttle_ = 0.1;
        state = 1;
        //compute steering using Audi library and store the value in steering_ (private data member)
        Audi audi;
        audi.computeSteering(currentOdo_, goals_.at(i), steering_, originalDistance);
        //Display goal point in gazebo sim (rviz)
        unsigned int j=0;
        pfms::geometry_msgs::Goal goal{j++,goals_.at(i)};
        pfmsConnectorPtr_->send(goal);
        while (!goalReached){
            //takes in brake, steering and throttle values and inputs to Ackerman cmd, sending commands through pfmsConnector and reads current odo
            pfms::commands::Ackerman cmd {repeats,brake_,steering_,throttle_};
            pfmsConnectorPtr_->send(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); //wait for a period to allow information to be sent then retrieved

            pfmsConnectorPtr_->read(currentOdo_,platformType_);
            straightDistToCurrentGoal = sqrt(pow(currentOdo_.position.x - goals_.at(i).x, 2) + pow(currentOdo_.position.y - goals_.at(i).y, 2));
            switch (state){
                case 0:
                    brake_ = 0.0;
                    steering_ = 0.0;
                    throttle_ = 0.0;
                    goalReached = true;
                case 1: //start driving, until distance to goal is less than 1m, then switch to apply brakes
                    if (straightDistToCurrentGoal < 1.0){
                        state = 2;
                    }
                    break;
                case 2: //apply brakes until close at goal
                    throttle_ = 0;
                    brake_ = 3000; //less than max braking, to just gradually slow down
                    if (straightDistToCurrentGoal < goalTolerance_+0.2){
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
            distanceTravelled_ += originalDistance - distanceTravelled_ - distanceToGoal();
            timeTravelled_ = distanceTravelled_/currentOdo_.linear.x;
            
            if (straightDistToCurrentGoal >= goalTolerance_){ //incremental counter for sending commands (needs increasing sequence counter)
                repeats++;
            }        
        }
    }
    platformStatus_ = pfms::PlatformStatus::IDLE;
    pfmsConnectorPtr_->send(platformStatus_);
}

// #include "ackerman.h"
// #include <chrono>   // Includes the system clock
// #include <algorithm> //Can use algorithms on STL containers
// #include <iostream>
// #include <thread>
// #include "audi.h"

// Ackerman::Ackerman(){
//     platformType_ = pfms::PlatformType::ACKERMAN;
//     pfmsConnectorPtr_ = std::make_shared<PfmsConnector>();
//     MAX_BRAKE_TORQUE = 8000.0; //Nm
//     DEFAULT_THROTTLE = 0.1; //m/s
//     i_ = 1;
//     brake_ = 0.0;
//     steering_ = 0.0;
//     throttle_ = 0.0;
//     goalTolerance_ = 0.5; //default goal tolerance for when unit tests do not set it.
// }

// Ackerman::~Ackerman(){

// }

// bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal, double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose){
//     //uses Audi library which 'automatically' computes values for distance, time and etimated goal pose and if the goal can be reached by Ackerman.
//     Audi audi;
//     if (audi.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose)){
//         return true;
//     }
//     else {
//         return false;
//     }
// }

// void Ackerman::run(void){
//     platformStatus_ = pfms::PlatformStatus::RUNNING;
//     std::thread ackermanRun(&Ackerman::reachGoals, this);
//     ackermanRun.detach();
// }

// void Ackerman::reachGoals(void){
//     bool goalReached;
//     unsigned long repeats;
//     int state;
//     double originalDistance;
//     pfmsConnectorPtr_->send(platformStatus_);
//     for (int i=0; i<goals_.size(); i++){
//         goalReached = false;
//         //Initialise (or reset) conditions for new reach goal command
//         repeats = 1;
//         throttle_ = 0.1;
//         state = 1;
//         //compute steering using Audi library and store the value in steering_ (private data member)
//         Audi audi;
//         audi.computeSteering(currentOdo_, goals_.at(i), steering_, originalDistance);
//         //Display goal point in gazebo sim (rviz)
//         unsigned int j=0;
//         pfms::geometry_msgs::Goal goal{j++,goals_.at(i)};
//         pfmsConnectorPtr_->send(goal);
//         while (!goalReached){
//             //takes in brake, steering and throttle values and inputs to Ackerman cmd, sending commands through pfmsConnector and reads current odo
//             pfms::commands::Ackerman cmd {repeats,brake_,steering_,throttle_};
//             pfmsConnectorPtr_->send(cmd);
//             std::this_thread::sleep_for(std::chrono::milliseconds(10)); //wait for a period to allow information to be sent then retrieved

//             pfmsConnectorPtr_->read(currentOdo_,platformType_);
//             distanceToCurrentGoal_ = sqrt(pow(currentOdo_.position.x - goals_.at(i).x, 2) + pow(currentOdo_.position.y - goals_.at(i).y, 2));
//             switch (state){
//                 case 0:
//                     brake_ = 0.0;
//                     steering_ = 0.0;
//                     throttle_ = 0.0;
//                     goalReached = true;
//                 case 1: //start driving, until distance to goal is less than 1m, then switch to apply brakes
//                     if (distanceToCurrentGoal_ < 1.0){
//                         state = 2;
//                     }
//                     break;
//                 case 2: //apply brakes until close at goal
//                     throttle_ = 0;
//                     brake_ = 3000; //less than max braking, to just gradually slow down
//                     if (distanceToCurrentGoal_ < goalTolerance_+0.2){
//                         state = 3;
//                     }
//                     break;
//                 case 3: //apply max braking torque to come to a complete stop
//                     brake_ = MAX_BRAKE_TORQUE;
//                     if(currentOdo_.linear.x <= 0 && currentOdo_.linear.y <= 0){ //check for when velocity is 0 (stopped) before proceeding with next goal or terminating (finishing) program
//                         state = 0;
//                     }
//                     break;
//             }
            
//             if (distanceToCurrentGoal_ >= goalTolerance_){ //incremental counter for sending commands (needs increasing sequence counter)
//                 repeats++;
//             }        
//         }
//     }
//     platformStatus_ = pfms::PlatformStatus::IDLE;
//     pfmsConnectorPtr_->send(platformStatus_);
// }