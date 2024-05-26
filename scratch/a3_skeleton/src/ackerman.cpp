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

    tolerance_ = 0.5;//We set tolerance to be default of 0.5

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
    
    if(pfmsGoals_.size()>0){
        // std::cout<<"Goals \n";
        goalReached = false;
        throttle_.data = 0.25;
        brake_.data = 0.0;
        state = 1;
        while (!goalReached){ //start driving to goal until goalReached returns true
            //compute steering using Audi library and store the value in steering_ (private data member)
            audi.computeSteering(getOdometry(), pfmsGoals_.front(), steering_.data, distanceToCurrentGoal_);
            // std::cout<<"State: "<<state<<", CMD: "<<steering_.data<<", "<<brake_.data<<", "<<throttle_.data<<std::endl;
            // std::cout<<"State: "<<state<<std::endl;
            switch (state){
                case 0: //state 0 is for when the platform is stopped and is waiting for either next goal or has finished mission goals
                    //set all command variables to 0 and goal has been reached, ending the loop for this current goal and moving to the next one
                    brake_.data = 0.0;
                    steering_.data = 0.0;
                    throttle_.data = 0.0;
                    goalReached = true;
                    pfmsGoals_.pop_front();
                    break;
                case 1: //start driving, until distance to goal is less than 2m, then switch to apply brakes
                    if (distanceToGoal() < 2){ //distanceToGoal is updated constantly through running chekcOriginToDestination, which updates the variable that distanceToGoal returns
                        state = 2; //switch to next state (braking)
                    }
                    break;
                case 2: //apply brakes until close at goal
                    throttle_.data = 0; //set throttle to 0, otherwise platform would be braking and accelerating, cancelling one or the other out
                    brake_.data = 3800; //less than max braking, to just gradually slow down
                    if (distanceToGoal() < tolerance_){ //check for when distance reaches the tolerance value set
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
            // std::cout<<"Publish Cmd \n";
            brakePub_->publish(brake_);
            steeringPub_->publish(steering_);
            throttlePub_->publish(throttle_);
        }
    }
    else{
        std::cout<<"Finished \n";
    }
}

void Ackerman::produceMarker(std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> road){
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "pairs";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = road.first;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 2.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "pairs";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = road.second;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 2.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);

    markerPub_->publish(marker_array);
}

void Ackerman::threadFunction(){
    double dx1, dy1, dx2, dy2;
    std::pair<double, double> euDist;
    float tolerance = 1;
    double estimatedRoadDist = 4;
    bool goalReachable = false;
    while(rclcpp::ok()){
        if(startMission_){
            std::cout<<"Mission Running, Road Size: "<<road_.size()<<std::endl;
            for (size_t j=0; j<road_.size(); j++){
                produceMarker(road_.at(j));
                std::cout<<"Checking Road \n";
                dx1 = road_.at(j).first.x - pfmsGoals_.front().x;
                dy1 = road_.at(j).first.y - pfmsGoals_.front().y;
                dx2 = road_.at(j).second.x - pfmsGoals_.front().x;
                dy2 = road_.at(j).second.y - pfmsGoals_.front().y;
                euDist.first = std::hypot(dx1,dy1);
                euDist.second = std::hypot(dx2,dy2);
                // std::cout<<"Next goal x: "<<pfmsGoals_.front().x<<", goal y: "<<pfmsGoals_.front().y<<std::endl;
                // std::cout<<"Cone 1: "<<road_.at(j).first.x<<" and y: "<<road_.at(j).first.y<<std::endl;
                // std::cout<<"Cone 2: "<<road_.at(j).second.x<<" and y: "<<road_.at(j).second.y<<std::endl;
                // std::cout<<"Road Side 1: "<<euDist.first<<" and Road Side 2: "<<euDist.second<<std::endl;
                if(std::abs(estimatedRoadDist - euDist.first) < tolerance && std::abs(estimatedRoadDist - euDist.second) < tolerance){
                    std::cout<<"Goal is between road cones \n";
                    goalReachable = true;
                    break;
                }
            }
            if(goalReachable){
                reachGoals();
                goalReachable = true;
            }else{
                std::cout<<"Goal cannot be reached \n";
                startMission_ = false;}
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