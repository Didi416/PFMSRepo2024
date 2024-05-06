#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan)
{
}



//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countObjectReadings()
{
    unsigned int count = 0;
    for (unsigned int i=0; i<laserScan_.ranges.size(); i++) {
        if (laserScan_.ranges.at(i) < laserScan_.range_max) {
            count++;
            
        }
    }
    return count;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countSegments()
{
    unsigned int count = 0;
    geometry_msgs::msg::Point point1;
    geometry_msgs::msg::Point point2;
    double euDist;
    double dx;
    double dy;
    std::vector<int> currentSeg;

    for (unsigned int i=0; i<laserScan_.ranges.size(); i++) {
        if (laserScan_.ranges.at(i) < laserScan_.range_max) {
            if (count == 0){ //if no other obstacle points have been recorded
                count++;
                currentSeg.push_back(i);
            }
            else{

                point1 = polarToCart(i-1);
                point2 = polarToCart(i);
                dx = point1.x - point2.x;
                dy = point1.y - point2.y;

                euDist = std::hypot(dx,dy);
                if (euDist < 0.3){
                    currentSeg.push_back(i);
                }
                else{
                    count++;
                    obstacles_.push_back(currentSeg);
                    currentSeg.clear();
                }
            }
        }
    }
    return count;
}

//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
geometry_msgs::msg::Point LaserProcessing::detectClosestCone(){
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint;
    // float avgX;
    // float avgY;
    float avgDist;
    countSegments();
    
    float closestCone = laserScan_.range_max;
    for (size_t i=0; i<obstacles_.size(); i++){

        tempPoint = segmentToPoint(i);
        avgDist = std::hypot(tempPoint.x, tempPoint.y);

        if (avgDist < closestCone){
            point.x = tempPoint.x;
            point.y = tempPoint.y;
            point.z = 0.0;
            closestCone = avgDist;
        }
    }

    return point;
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
geometry_msgs::msg::Point LaserProcessing::detectRoadCentre(){

    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint1;
    // geometry_msgs::msg::Point closestPointsPair;
    double dx;
    double dy;
    double euDist;
    float tolerance = 0.3;
    double roadWidth = 8; //roughly 8m

    geometry_msgs::msg::Point closestPoint = detectClosestCone();
    // std::cout<<"Start \n";

    for (size_t i=0; i<obstacles_.size(); i++){
        tempPoint1 = segmentToPoint(i);
        dx = closestPoint.x - tempPoint1.x;
        dy = closestPoint.y - tempPoint1.y;
        euDist = std::hypot(dx,dy);
        if (std::abs(roadWidth-euDist) < tolerance){
            // std::cout<<"Road found \n";
            // std::cout<<"EuDist"<<euDist<<std::endl;
            // std::cout<<"closestPoint X: "<<closestPoint.x<<" and Y: "<<closestPoint.y<<std::endl;
            // std::cout<<"tempPoint X: "<<tempPoint1.x<<" and Y: "<<tempPoint1.y<<std::endl;
            break;
        }
    }

    point.x = (std::abs(closestPoint.x) + std::abs(tempPoint1.x))/2;
    point.y = (std::abs(closestPoint.y) + std::abs(tempPoint1.y))/2;
    // std::cout<<"X: "<<point.x<<" and Y: "<<point.y<<std::endl;

    return point;
}

void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    laserScan_=laserScan;
    cones_.clear();
}


geometry_msgs::msg::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::msg::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

geometry_msgs::msg::Point LaserProcessing::segmentToPoint(int i){

    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint;
    float avgX = 0.0;
    float avgY = 0.0;
    
    for (size_t j=0; j<obstacles_.at(i).size(); j++){
        tempPoint = polarToCart(obstacles_.at(i).at(j));
        avgX += tempPoint.x;
        avgY += tempPoint.y;
    }
    avgX = avgX/obstacles_.at(i).size();
    avgY = avgY/obstacles_.at(i).size();

    point.x = avgX;
    point.y = avgY;
    point.z = 0.0;
    return point;
}