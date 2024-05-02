#include "controller.h"
#include <chrono>   // Includes the system clock
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include <thread>

Controller::Controller(){
    //initialise all distance and time variables to 0
    distanceTravelled_ = 0;
    timeTravelled_ = 0;
    distanceToCurrentGoal_; 
    timetoCurrentGoal_;

    goalTolerance_ = 0.5; //default goal tolerance for when unit tests do not set it.
    //threading
    running_ = false; //initialise check condition to run reachGoals
    threads_.push_back(std::thread(&Controller::reachGoals, this)); //start thread and add to vector for easy destruction of objects and threads
    platformStatus_ = pfms::PlatformStatus::RUNNING; //set platform status to RUNNING
}

Controller::~Controller(){ //object destructor
    //set running_ to false, as mission is complete, platform object is no longer used
    running_ = false;
    //join threads to parent to terminate
    for(auto & thread : threads_){
        thread.join();
    }
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    goals_ = goals;
    pfms::nav_msgs::Odometry origin = getOdometry();
    for (auto goal:goals_){
        //For checking the output of the odomtery readings:
        // std::cout<<"Current Odo Readings: "<<std::endl;
        // std::cout<<origin.position.x<<std::endl;
        // std::cout<<origin.position.y<<std::endl;
        // std::cout<<origin.position.z<<std::endl;
        // std::cout<<origin.yaw<<std::endl;

        //Use checkOriginToDestination (implemented in derived class) to check that the goals can be reached and the distances/times of each
        checkOriginToDestination(origin, goal, distanceToCurrentGoal_, timetoCurrentGoal_, estimatedGoalPose_);
        origin = estimatedGoalPose_; //for travelling to each goal from the previous, the origin needs to be updated to be the previous goal's estimate pose, not the current origin of the platform
    }
    return true;
}

void Controller::run(void){
    //when function is calles, conditional variable is set to true and notifies all waiting threads (i.e. reachGoals)
    running_ = true;
    std::unique_lock<std::mutex> lck(mtxStart_);
    mtxStart_.unlock();
    cvStart_.notify_all(); //"wakes up" the sleeping cv.wait, now that the running_ variable has been set to true
    pfmsConnectorPtr_->send(platformStatus_);
}

double Controller::distanceToGoal(void){ // Returns the distance to current goal as claculated in checkOriginToDestinatione(), continuously updated through reachGoals
    return distanceToCurrentGoal_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}

double Controller::timeToGoal(void){ // Returns the time to current goal as claculated in checkOriginToDestination(), continuously updated through reachGoals
    return timetoCurrentGoal_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}

double Controller::distanceTravelled(void){//Returns total distance travelled up to the current point in time (updates when called)
    return distanceTravelled_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}

double Controller::timeTravelled(void){//Calculates total time in motion up to the current point in time (updates when called)
    return timeTravelled_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}

pfms::PlatformType Controller::getPlatformType(void){ //Returns the platform type that was created, either Ackerman or Quadcopter
    return platformType_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}

pfms::PlatformStatus Controller::status(void){ //Returns platform's current status (IDLE, RUNNING, TAKEOFF/LANDING)
    return platformStatus_; //protected variable so can be accessed from other functions in classes in inheritance tree (base and derived)
}

bool Controller::setTolerance(double tolerance){ //sets acceptable goal tolerance when driving platform to reach goals
    goalTolerance_ = tolerance; //stores a copy in a protected data member for access throughout program
    return true; //returns true that tolerance is set
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){ //Returns the current Odometry of the platform, including global position, and linear velocity
    pfmsConnectorPtr_->read(currentOdo_,platformType_); //uses pfms pipes to read the simulator's odometry of the specified platform, stores the value in currentOdo_ to be used throughout program
    return currentOdo_;
}

std::vector<pfms::geometry_msgs::Point> Controller::getObstacles(void){ //Used for SUPER for the Quadcopter to get the obstacles in the world and hence the path the Audi has to take
    std::vector<pfms::geometry_msgs::Point> obstacles;
    return obstacles;
}

bool Controller::updateDistTime(double velocity){ //updates total distance and time variables continously (called in reachGoals)
    getOdometry(); //update odometry
    //calculate distance travelled since previous update (very small increment so the straight distance between points is sufficient, even for Ackerman)
    distanceTravelled_ += sqrt(pow((currentOdo_.position.x - previousOdo_.position.x),2)+pow((currentOdo_.position.y - previousOdo_.position.y),2));
    //Update time travelled by taking ditance over current velocity of the platform (caluculated in reachGoals and input into the function)
    timeTravelled_ = distanceTravelled_/velocity;
    previousOdo_ = currentOdo_; //update previous odometry to current odometry for next update
    return true;
}