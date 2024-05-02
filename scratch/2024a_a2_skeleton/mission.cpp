#include "mission.h"
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>
#include "controller.h"
#include <queue>
#include <set>

Mission::Mission(std::vector<ControllerInterface*> controllers){ //Mission class constructor
    controllers_ = controllers; //copy controllers to private data member to be access from whole class
    //resize totalMissionDistance, totalMissionTime and status vectors to accomodate all controllers (either 1, 2, 3... etc)
    totalMissionDistance_.resize(controllers.size()); 
    totalMissionTime_.resize(controllers.size());
    status_.resize(controllers.size());
}

Mission::~Mission(){ //Mission class destructor

}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform){
    missionGoals_ = goals; //copy current order of goals into private data member
    int a = 0; //initialise int variable to be able to identify position of controller (as specified from the 'platform' input variable) in the controllers_ vector
    for (int i=0; i<controllers_.size(); i++){ //iterate through active controllers
        if (controllers_.at(i)->getPlatformType() == platform){ //check for which controller in the vector is the same type as specified in the input 'platform' variable
            a = i; //store the position of the controller in the vector
            break; //no need to keep searching, as we have already found the controller
        }
    }
    // Initialise setGoals variable
    pfms::nav_msgs::Odometry origin, estimatedGoalPose; //odometry readings
    origin = controllers_.at(a)->getOdometry(); //update current odometry of the platform
    double dist, time; //for input into chekOriginToDestination function that calculates distance

    //Variables for ADVANCED
    std::vector<int> order(missionGoals_.size(),0); //specif
    AdjacencyList graph;
    std::vector<pfms::geometry_msgs::Point> tempGoals;

    switch (objective_){ //check mission objective (BASIC, ADVANCED or SUPER)
        case mission::Objective::BASIC: //for BASIC mode
            controllers_.at(a)->setGoals(missionGoals_); //setGoals for the controller
            break;
        case mission::Objective::ADVANCED: //for ADVANCED mode
            graph = generateGraph(a);//generate a graph fo the goals, to determine whcih goals can be accessed from other goals
            order = bestPathSearch(graph); //search the graph for the shortest path through all goals
            //following for loop temporarily stores the mission goals in the order specified from teh bestPathSearch
            for (int i=0; i<order.size(); i++){
                tempGoals.push_back(missionGoals_.at(order.at(i)));
            }
            missionGoals_.assign(tempGoals.begin(), tempGoals.end()); //updates the private goals vector with the temporary goals vector, updating to correct order
            controllers_.at(a)->setGoals(missionGoals_); //setGoals in correct order to the controller
            break;
        case mission::Objective::SUPER: //for SUPER (incomplete)
            controllers_.at(a)->setGoals(missionGoals_);
            break;
    }
    //same process for all modes
    for(int i=0; i<missionGoals_.size(); i++){ //iterate through goals
        platGoalAssoc_.push_back(std::make_pair(a,order.at(i))); //assign each goal to the controller, accordign to the correct order
        controllers_.at(a)->checkOriginToDestination(origin, missionGoals_.at(i), dist, time, estimatedGoalPose); //use checkOriginToDestination to compute distances between goals
        origin = estimatedGoalPose; //update origin to be the estimated pose of the goal that the distance was just calculated for, instead of always from currect platform origin
        totalMissionDistance_.at(a) += dist; //add up the distances between goals to get the total distance of the mission (needed for status)
    }
} 

bool Mission::run(){ //Starts mission running
    for (auto controller : controllers_){ //iterate through all active controllers
        controller->run(); //call their respective run functions, therefore starting the threads within each object to reachGoals
    } //Mission::run terminates (non-blocking) as threading has taken over for the reachGoals in each controller, and so reaches the end of Mission::run()
    return true;
}

std::vector<unsigned int> Mission::status(void){ //
    for (int i=0;i<controllers_.size(); i++){ //iterate through controllers
        double percentage = (getDistanceTravelled().at(i)/totalMissionDistance_.at(i))*100; //calculate percentage of mission distance currently completed
        if(controllers_.at(i)->status() == pfms::PlatformStatus::IDLE){ //check if platform status is set to IDLE (platofrm has reached all goals)
            percentage = 100; //set percentage to 100, as sometimes percentages can vary (not exactly 100, more often around 95-97%)
        }
        if(percentage >= 100 && controllers_.at(i)->status() != pfms::PlatformStatus::IDLE){ //check if percentage goes above 100 but all goals have not been reached yet (not yet IDLE)
            percentage = 99; //keep percentage at 99%
        }
        status_.at(i) = percentage; //allocate the 'i-th' controller's distance percentage to the controller's status position in the vector
    }
    return status_; //returns the vector
}

void Mission::setMissionObjective(mission::Objective objective){
    objective_ = objective; //return the selected objective of the mission, either BASIC, ADVANCED or SUPER
}

std::vector<double> Mission::getDistanceTravelled(){ //function to return current distance of the whole mission (multiple goals)
    std::vector<double> distanceTravelled; //initialise vector of doubles to store the total distance travelled at this moment of all controllers
    for (int i=0;i<controllers_.size(); i++){ //iterate through controllers_ vector (number of controllers)
        distanceTravelled.push_back(controllers_.at(i)->distanceTravelled()); //push_back value of total distance travelled to this moment as calculated in the derived classes of Controller
    }
    return distanceTravelled;
}

std::vector<double> Mission::getTimeMoving(){ //function to return current time taken in motion of the whole mission (multiple goals)
    std::vector<double> timeMoving; //initialise vector of doubles to store the total time travelled at this moment of all controllers
    for (int i=0;i<controllers_.size(); i++){ //iterate through controllers_ vector (number of controllers)
        timeMoving.push_back(controllers_.at(i)->timeTravelled()); //push_back value of total distance travelled to this moment as calculated in the derived classes of Controller
    }
    return timeMoving; //return the vector of total time in motion
}

std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation(){  
    return platGoalAssoc_; //returns the vector of pairs relating each of the goals to the controller platform
}

std::vector<int> Mission::bestPathSearch(AdjacencyList graph){ // Search the goals graph to find shortest path
    //start tsp search, focusing on moving to the goal with the next shortest distance from current goal
    std::vector<int> order;
    //List all goals in a vector, to do permutations
    std::vector<int> goals;
    for (unsigned int i=0; i< graph.size(); i++){
        goals.push_back(i);
    }
    double minDistance = 1e6; //Set initial minimum distance to a large value, so first permutation is definitely less than

    //Go through all permutations of the order of goals, comparing total distances to 
    //determine the minimum distance possible and asociated permuation:
    do {
        bool OK = true; //Use this to abort search if distanec is already above minimum distance
        unsigned int i = 1;//Start from index 1 to end 
        double dist = 0; // Current distance travelled through goals
        while((i<goals.size()) && (OK)){
            // Checks distance between two goals in the order dictated by the current permutation
            unsigned int goal1 = goals.at(i-1);
            unsigned int goal2 = goals.at(i);
            //We find in the adjacency list (graph), the goal1 connection to goal2 and access the second element which is the distance between goal1 and goal2
            dist += graph.at(goal1).at(goal2).second;
            //We abort current search if dist is already over the min distance
            if(dist > minDistance){
                OK = false;
            }
            i++; // increment to next goal in permutation
        }
        dist += distancesFromOrigin_.at(goals.at(0));
        if(dist<minDistance){
            minDistance=dist; // Save current dist as new minimum distance
            order.clear(); // Clear the current order of goals
            order=goals; // Save the order of goals as current shortest path
        }
    } 
    while (std::next_permutation(goals.begin(), goals.end())); //Go to next permutation of goals order

    return order;
}

AdjacencyList Mission::generateGraph(int controller){ //Generate graph for searching from mission goals
    AdjacencyList graph (missionGoals_.size()); // Create graph of size of mission goals (number of vectors in the vector = missionGoals.size())
    distancesFromOrigin_.resize(missionGoals_.size()); //Resize private data member vector of distances from the current platform origin to fit the number of goals
    double dist, time; // for calculating distance and time in checkOriginToestination function
    pfms::nav_msgs::Odometry origin, estimatedGoalPose; //stores origin or starting position for calculating checkOriginToDestination function
    origin = controllers_.at(controller)->getOdometry(); //initially set origin to platform's current odometry to get distances from origin to all goals
    for (int j=0; j<missionGoals_.size(); j++){ //iterate through goals
        //calculate distance from current platform odometry to the current goal (to be used in identifying shortest path)
        controllers_.at(controller)->checkOriginToDestination(origin, missionGoals_.at(j), distancesFromOrigin_.at(j), time, estimatedGoalPose);
    }
    for (int i=0; i<missionGoals_.size(); i++){ //iterate through goals
        // store current goal point as the current origin to calculate the distances from the current goal to all other goals
        origin.position.x = missionGoals_.at(i).x;
        origin.position.y = missionGoals_.at(i).y;
        origin.position.z = missionGoals_.at(i).z;
        origin.yaw = origin.yaw;
        origin.linear.x = 0;
        origin.linear.y = 0;
        for (int j=0; j<missionGoals_.size(); j++){ //iterate through goals
            if(controllers_.at(controller)->checkOriginToDestination(origin, missionGoals_.at(j), dist, time, estimatedGoalPose)){ //checks if goal can be reached from another goal position
                dist = sqrt(pow(origin.position.x - missionGoals_.at(j).x, 2) + pow(origin.position.y - missionGoals_.at(j).y, 2)); //calculates straight line distance to goals, as Audi library doesn't always calculate the distance correctly
                graph.at(i).push_back(std::make_pair(j,dist)); //add an entry into the graph, to signify that the goal 'j' can be reached from goal 'i' in a distance calculated by 'dist'
            }
            else{ //if a goal cannot be reached from current goal position (e.g. Ackerman cannot reach the goal it is already at)
                graph.at(i).push_back(std::make_pair(j,0)); //still ad an entry, but make the distance equal to 0
            }
        }
    }
    return graph;
}