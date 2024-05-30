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

    tolerance_ = 0.5;//set tolerance to be default of 0.5

    brakePub_  = this->create_publisher<std_msgs::msg::Float64>("orange/brake_cmd",3);  
    steeringPub_ = this->create_publisher<std_msgs::msg::Float64>("orange/steering_cmd",3); 
    throttlePub_ = this->create_publisher<std_msgs::msg::Float64>("orange/throttle_cmd",3); 

    thread_ = new std::thread(&Ackerman::threadFunction, this);
}

Ackerman::~Ackerman(){

}

void Ackerman::reachGoals(void){
    // std::cout<<"ReachGoals \n";
    bool goalReached; //checks for when a goal has been reached and controls the while loop
    int state; //state of the platform, either starting movment to goal (set throttle), slowing down, full brake force, and when reached goal
    double distanceToCurrentGoal;
    
    if(pfmsGoals_.size()>0){
        std::cout<<"Goals \n";
        goalReached = false;
        throttle_.data = 0.25;
        brake_.data = 0.0;
        state = 1;
        while (!goalReached){ //start driving to goal until goalReached returns true
            if(!startMission_){break;}
            //compute steering using Audi library and store the value in steering_ (private data member)
            audi.computeSteering(getOdometry(), pfmsGoals_.front(), steering_.data, distanceToCurrentGoal);
            // std::cout<<"State: "<<state<<", CMD: "<<steering_.data<<", "<<brake_.data<<", "<<throttle_.data<<std::endl;
            // std::cout<<"State: "<<state<<std::endl;
            switch (state){
                case 0: //state 0 is for when the platform is stopped and is waiting for either next goal or has finished mission goals
                    //set all command variables to 0 and goal has been reached, ending the loop for this current goal and moving to the next one
                    brake_.data = 0.0;
                    steering_.data = 0.0;
                    throttle_.data = 0.0;
                    pfmsGoals_.pop_front();
                    goalReached = true;
                    break;
                case 1: //start driving, until distance to goal is less than 2m, then switch to apply brakes
                    if (distanceToCurrentGoal < 2){ //distanceToGoal is updated constantly through running chekcOriginToDestination, which updates the variable that distanceToGoal returns
                        state = 2; //switch to next state (braking)
                    }
                    break;
                case 2: //apply brakes until close at goal
                    throttle_.data = 0; //set throttle to 0, otherwise platform would be braking and accelerating, cancelling one or the other out
                    brake_.data = 3500; //less than max braking, to just gradually slow down
                    if (distanceToCurrentGoal < tolerance_){ //check for when distance reaches the tolerance value set
                        state = 0; //switch to max braking to come to a stop
                    }
                    break;
                // case 3: //apply max braking torque to come to a complete stop
                //     brake_.data = MAX_BRAKE_TORQUE; //MAX_BRAKE_TORQUE is set to 8000Nm as per specifications
                //     if(currentOdo_.linear.x <= 0 && currentOdo_.linear.y <= 0){ //check for when velocity is 0 (stopped) before proceeding with next goal or terminating (finishing) program
                //         state = 0; //default state, command variables set to 0 (platform does not move)
                //     }
                //     break;
            }
            brakePub_->publish(brake_);
            steeringPub_->publish(steering_);
            throttlePub_->publish(throttle_);
        }
    }
}

void Ackerman::threadFunction(){
    while(rclcpp::ok()){
        if(startMission_){
            std::cout<<"Next goal x: "<<pfmsGoals_.front().x<<", goal y: "<<pfmsGoals_.front().y<<std::endl;
            reachGoals();
        }
        else{
            brake_.data = MAX_BRAKE_TORQUE;
            steering_.data = 0.0;
            throttle_.data = 0.0;
            brakePub_->publish(brake_);
            steeringPub_->publish(steering_);
            throttlePub_->publish(throttle_);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}