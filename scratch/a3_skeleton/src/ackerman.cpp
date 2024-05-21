#include "ackerman.h" //include header file
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include <thread>

Ackerman::Ackerman(){
    MAX_BRAKE_TORQUE = 8000.0; //Nm
    brake_ = std_msgs::msg::Float64();
    steering_ = std_msgs::msg::Float64();
    throttle_ = std_msgs::msg::Float64();
    velocity_ = 2.91;

    tolerance_ = 0.5;//We set tolerance to be default of 0.5

    brakePub_  = this->create_publisher<std_msgs::msg::Float64>("orange/brake_cmd",3);  
    steeringPub_ = this->create_publisher<std_msgs::msg::Float64>("orange/steering_cmd",3); 
    throttlePub_ = this->create_publisher<std_msgs::msg::Float64>("orange/throttle_cmd",3); 

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Ackerman::timerCallback, this));
}

Ackerman::~Ackerman(){

}

void Ackerman::reachGoals(void){
    
    brake_.data = 0;
    steering_.data = 0;
    throttle_.data = 0.1;
    brakePub_->publish(brake_);
    steeringPub_->publish(steering_);
    throttlePub_->publish(throttle_);

}

void Ackerman::timerCallback(){
    RCLCPP_INFO(this->get_logger(), "Ackerman Timer Callback");
    reachGoals();
}

// void Ackerman::reachGoals(void){
//     //checks for if conditional variable (running_) has been set to true, mutex is used for synchronisation and securing common data
//     // std::unique_lock<std::mutex> lck(mtxStart_);
//     // cvStart_.wait(lck, [&](){return running_==true;}); //wait function checks for boolean value (condition variable)
//     // while(running_){ //only when running_ is set ot true, do we proceed with reaching goals for Ackerman
//         //initialise variables to be used in the reachGoals scope
//         bool goalReached; //checks for when a goal has been reached and controls the while loop
//         unsigned long repeats; //used for sending command to pfms pipes, command needs incrementing counter to move platform
//         int state; //state of the platform, either starting movment to goal (set throttle), slowing down, full brake force, and when reached goal
//         double prevDistance; //placeholder for distance for when we need to calculate steering (computeSteering function in Audi library)
//         // pfmsConnectorPtr_->send(platformStatus_); //sends current platform status to pfms pipes (should be set to RUNNING)
//         // pfms::commands::Ackerman cmd; //creates pfms pipes command object for Ackerman, only need to create object once, not every loop
//         // for (int i=0; i<goals_.size(); i++){ //iterate through goals to print goal points in Gazebo RVIZ
//         //     unsigned int j=0;
//         //     pfms::geometry_msgs::Goal goalAck{j++,goals_.at(i)}; //set goal
//         //     pfmsConnectorPtr_->send(goalAck); //send goal to pfms pipes
//         // }
//         for (int i=0; i<goals_.size(); i++){ //iterate through goals
//             goalReached = false; //set goalReached bool to false at the start of the new goal
//             //Initialise (or reset) conditions for new reach goal command
//             repeats = 1;
//             throttle_ = 0.2;
//             state = 1;
//             //compute steering using Audi library and store the value in steering_ (private data member)
//             // Audi audi;
//             // audi.computeSteering(currentOdo_, goals_.at(i), steering_, prevDistance);
//             //update distance and time variable
//             // checkOriginToDestination(currentOdo_,goals_.at(i),distanceToCurrentGoal_,timetoCurrentGoal_,estimatedGoalPose_);
//             //Display goal point in gazebo sim (rviz)
//             while (!goalReached){ //start driving to goal until goalReached returns true
//                 getOdometry(); //update odometry of platform every loop (for use of currentOdo_ variable, 
//                 //instead of calling getodomtery everytime, updating at the start of each loop is sufficient)
//                 //cmd takes in brake, steering and throttle values and inputs to Ackerman cmd, sending commands through pfmsConnector
//                 // cmd = {repeats,brake_,steering_,throttle_};
//                 // pfmsConnectorPtr_->send(cmd);
//                 // std::this_thread::sleep_for(std::chrono::milliseconds(10)); //wait for a period to allow information to be sent then retrieved
//                 // checkOriginToDestination(currentOdo_,goals_.at(i),distanceToCurrentGoal_,timetoCurrentGoal_,estimatedGoalPose_); //recalculate distance and time for each loop
//                 // pfmsConnectorPtr_->read(currentOdo_,platformType_);
//                 switch (state){
//                     case 0: //state 0 is for when the platform is stopped and is waiting for either next goal or has finished mission goals
//                         //set all command variables to 0 and goal has been reached, ending the loop for this current goal and moving to the next one
//                         brake_ = 0.0;
//                         steering_ = 0.0;
//                         throttle_ = 0.0;
//                         goalReached = true;
//                     case 1: //start driving, until distance to goal is less than 2m, then switch to apply brakes
//                         if (distanceToGoal() < 2){ //distanceToGoal is updated constantly through running chekcOriginToDestination, which updates the variable that distanceToGoal returns
//                             state = 2; //switch to next state (braking)
//                         }
//                         break;
//                     case 2: //apply brakes until close at goal
//                         throttle_ = 0; //set throttle to 0, otherwise platform would be braking and accelerating, cancelling one or the other out
//                         brake_ = 5800; //less than max braking, to just gradually slow down
//                         if (distanceToGoal() < goalTolerance_){ //check for when distance reaches the tolerance value set
//                             state = 3; //switch to max braking to come to a stop
//                         }
//                         break;
//                     case 3: //apply max braking torque to come to a complete stop
//                         brake_ = MAX_BRAKE_TORQUE; //MAX_BRAKE_TORQUE is set to 8000Nm as per specifications
//                         if(currentOdo_.linear.x <= 0 && currentOdo_.linear.y <= 0){ //check for when velocity is 0 (stopped) before proceeding with next goal or terminating (finishing) program
//                             state = 0; //default state, command variables set to 0 (platform does not move)
//                         }
//                         break;
//                 }
//                 velocity_ = sqrt(pow(currentOdo_.linear.x, 2)+pow(currentOdo_.linear.y, 2)); //calculate valocity of the platform by reading from current odometry
//                 updateDistTime(velocity_); //function updates total distance and time travelled from start to current time. 
                
//                 if (distanceToGoal() <= goalTolerance_){ //check for when distance tolerance value has been reached
//                     if (i == goals_.size()-1){ //check if all goals have been completed, (for loop has run for each goal/at end of mission)
//                         platformStatus_ = pfms::PlatformStatus::IDLE; //set platform status to IDLE when finished mission goals 
//                     }
//                 }else{repeats++;} //incremental counter for sending commands (needs increasing sequence counter)
//             }
//         }
//         // platformStatus_ = pfms::PlatformStatus::IDLE;
//         // pfmsConnectorPtr_->send(platformStatus_);
//         // std::this_thread::sleep_for (std::chrono::milliseconds(10));
//     }
// // }