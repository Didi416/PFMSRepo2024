#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"
#include <vector>
#include "pfms_types.h"
#include <iostream>

using std::vector;
using std::cout;
using std::endl;

int main(int argc, char *argv[]) {

    // double x=3;
    // double y=-3;

    // if(argc !=3){
    //     cout << " Not arguments given on command line." << endl;
    //     cout << " usage: " << argv[0] << "<x> <y>" << endl;
    //     cout << " defaulting to: " << x << " " << y << endl;
    // }
    // else{
    //     x = atof(argv[1]);
    //     y = atof(argv[2]);
    //     cout << " Goal : " << x << " " << y << endl;        
    // }


    //We create a vector of controllers and add the Ackerman controller to it
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=10,y=0,z=0;
    pfms::geometry_msgs::Point pt1{10,0,0};
    //We set a tolerance for the goal reaching
    controllers.front()->setTolerance(0.5);

    //We set the goal for the Ackerman controller, check it's reachable and get the distance and time to reach the goal
    bool reachable = controllers.at(0)->setGoal(pt1);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;
    


    //This now triggers the Ackerman to reach the goal and "blocks" until the goal is reached
    bool reached = controllers.at(0)->reachGoal();


    return 0;
}
