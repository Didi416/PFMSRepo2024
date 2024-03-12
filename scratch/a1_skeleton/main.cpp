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
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new SkidSteer());


    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);
    controllers.at(1)->setTolerance(0.5);

        //Goals
    pfms::geometry_msgs::Point goal0{10,3};
    pfms::geometry_msgs::Point goal1{0,-3};

    std::vector<pfms::geometry_msgs::Point*> goals;
    goals.push_back(&goal0);
    goals.push_back(&goal1);


    //////////////////////////////////////////////////////////////////////////////////
    // Let's now check missions
    Mission mission(controllers);
    mission.setGoals(goals);

    mission.setMissionObjective(mission::Objective::TIME);

    std::vector<unsigned int> assignment =  mission.getPlatformGoalAssociation();

    for(unsigned int i=0;i<assignment.size();i++){
        std::cout << i << " : " << assignment.at(i) << std::endl;
    }

    bool OK = mission.runMission();

    if(OK){
        std::cout << "Controller possibly reached goal" << std::endl;
    }
    else {
        std::cout << "Controller CAN NOT reach goal" << std::endl;
    }

    //The and skidsteer should be within 0.5m of goal position when completing motion.


    return 0;
}
