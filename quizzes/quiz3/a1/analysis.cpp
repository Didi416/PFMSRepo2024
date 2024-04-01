#include "analysis.h"
#include "tf.h"
#include "tf2.h"

using std::vector;
using std::pair;
using geometry_msgs::Point;

Analysis::Analysis(std::vector<Point> goals) :
    goals_(goals)
{

}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
vector<double> Analysis::timeToImpact(Pose origin){

    //The consts you will need are
    //Display::OMEGA_MAX
    //Display::V_MAX

    vector<double> times;
    RangeBearingStamped rangeBearing;
    double angleToGoal = 0;
    double distance = 0;
    double rotationTime = 0;
    double driveTime = 0;
    double time = 0;

    for (int i=0; i<goals_.size(); i++){
        rangeBearing = tf2::global2local(goals_.at(i),origin);
        angleToGoal = rangeBearing.bearing;
        std::cout<<"Angle: "<<angleToGoal<<std::endl;
        if(angleToGoal < 0){
            rotationTime = angleToGoal/-Display::OMEGA_MAX;
        }
        else{
            rotationTime = angleToGoal/Display::OMEGA_MAX;
        }

        distance = rangeBearing.range;
        std::cout<<"Distance: "<<distance<<std::endl;
        driveTime = distance/Display::V_MAX;

        time = rotationTime + driveTime;
        std::cout<<"Time: "<<time<<std::endl;
        times.push_back(time);
    }

    return times;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
AdjacencyList Analysis::exportGraph(){
    //std::make_pair(odom.at(i), i));
    AdjacencyList graph(goals_.size());
    double distance = 0;
    double dx;
    double dy;
    RangeBearingStamped rangeBearing;

    for (int i = 0; i<goals_.size(); i++){
        for (int j = 0; j<goals_.size(); j++){
            if (i!=j){
                dx = goals_.at(i).x - goals_.at(j).x;
                dy = goals_.at(i).y - goals_.at(j).y;
                distance = sqrt(dx*dx + dy*dy);

                graph.at(i).push_back(std::make_pair(distance, j));
            }
        }
    }

    return graph;
}
