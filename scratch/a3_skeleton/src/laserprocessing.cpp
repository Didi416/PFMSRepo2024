#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <iostream>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan), objectReadings_(0)
{
}

unsigned int LaserProcessing::countObjectReadings()
{
    std::unique_lock<std::mutex> lck(mtx);
    sensor_msgs::msg::LaserScan laserScan = laserScan_;   
    lck.unlock();

    unsigned int count=0;
    std::vector<double> x;std::vector<double> y;
    for (unsigned int i = 0; i < laserScan_.ranges.size(); i++)
    {
        if ((laserScan_.ranges.at(i) > laserScan_.range_min) &&
            (laserScan_.ranges.at(i) < laserScan_.range_max) &&
            !isnan(laserScan_.ranges.at(i)) &&
            isfinite(laserScan_.ranges.at(i))){

            count++;
            objectReadingIndex_.push_back(i);
        }
    }

    objectReadings_=count;
    return objectReadings_;
}

unsigned int LaserProcessing::countSegments()
{
    unsigned int count = 0;
    geometry_msgs::msg::Point point1;
    geometry_msgs::msg::Point point2;
    double euDist;
    double dx;
    double dy;
    std::vector<int> currentSeg;
    obstacles_.clear();

    // for (unsigned int i=0; i<objectReadingIndex_.size(); i++) {
    //     if (laserScan_.ranges.at(objectReadingIndex_.at(i)) < laserScan_.range_max) {
    //         if (count == 0){ //if no other obstacle points have been recorded
    //             count++;
    //             currentSeg.push_back(i);
    //         }
    //         else{
    //             point1 = polarToCart(objectReadingIndex_.at(i-1));
    //             point2 = polarToCart(objectReadingIndex_.at(i));
    //             // std::cout<<"P1 x: "<<point1.x<<" y: "<<point1.y<<std::endl;
    //             // std::cout<<"P2 x: "<<point2.x<<" y: "<<point2.y<<std::endl;
    //             dx = point1.x - point2.x;
    //             dy = point1.y - point2.y;
    //             euDist = std::hypot(dx,dy);
    //             // std::cout<<"Before: "<<euDist<<std::endl;
    //             if (euDist < 0.3){
    //                 currentSeg.push_back(i);
    //             }
    //             else{
    //                 count++;
    //                 obstacles_.push_back(currentSeg);
    //                 currentSeg.clear();
    //                 std::cout<<euDist<<std::endl;
    //                 std::cout<<"dx: "<<dx<<" dy: "<<dy<<"Obs size:"<<obstacles_.size()<<std::endl;
    //             }
    //         }
    //     }
    // }

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
                    // std::cout<<"dx: "<<dx<<" dy: "<<dy<<"Obs size:"<<obstacles_.size()<<std::endl;
                }
            }
        }
    }
    return count;
}

std::vector<geometry_msgs::msg::Point> LaserProcessing::detectConeCentres(){
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint;
    std::vector<geometry_msgs::msg::Point> cones;
    double coneDist;
    double minConeDist = 1;
    bool addCone;

    countSegments();
    
    for (size_t i=0; i<obstacles_.size(); i++){
        tempPoint = segmentToPoint(i);

        point.x = tempPoint.x;
        point.y = tempPoint.y;
        point.z = 0.0;
        // std::cout<<"Px: "<<point.x<<" Py: "<<point.y<<std::endl;
        addCone = true;
        // for(size_t j=0; j<cones.size(); j++){
        //     coneDist = std::hypot((point.x - cones.at(j).x),(point.y - cones.at(j).y));
        //     if(coneDist < minConeDist){
        //         addCone = false;
        //         std::cout<<"Don't add cone \n";
        //     }
        // }
        if(addCone){
            cones.push_back(point);
            // std::cout<<"Add cone \n";
        }
    }
    return cones;
}

geometry_msgs::msg::Point LaserProcessing::detectClosestCone(){
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint;
    // float avgX;
    // float avgY;
    float avgDist;
    std::vector<geometry_msgs::msg::Point> cones = detectConeCentres();
    
    float closestCone = laserScan_.range_max;
    for (size_t i=0; i<cones.size(); i++){
        tempPoint = cones.at(i);
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

std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> LaserProcessing::detectRoad(){
    std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> road;
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint1;
    // geometry_msgs::msg::Point closestPointsPair;
    double dx;
    double dy;
    double euDist;
    float tolerance = 3;
    double roadWidth = 8; //roughly 8m

    geometry_msgs::msg::Point closestPoint = detectClosestCone();
    std::vector<geometry_msgs::msg::Point> cones = detectConeCentres();
    // std::cout<<"Start \n";

    for (size_t i=0; i<cones.size(); i++){
        tempPoint1 = cones.at(i);
        dx = closestPoint.x - tempPoint1.x;
        dy = closestPoint.y - tempPoint1.y;
        euDist = std::hypot(dx,dy);
        if (std::abs(roadWidth-euDist) < tolerance){
            // std::cout<<"Road found \n";
            // std::cout<<"EuDist"<<euDist<<std::endl;
            // std::cout<<"closestPoint X: "<<closestPoint.x<<" and Y: "<<closestPoint.y<<std::endl;
            // std::cout<<"tempPoint X: "<<tempPoint1.x<<" and Y: "<<tempPoint1.y<<std::endl;
            road.first = point;
            road.second = tempPoint1;
            break;
        }
    }

    return road;
}

void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    std::unique_lock<std::mutex> lck(mtx);
    laserScan_ = laserScan;    
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

double LaserProcessing::cartesianToPolar(geometry_msgs::msg::Point cart){
    // double range = (sqrt(cart.x * cart.x + cart.y * cart.y));
    double angle = (atan2(cart.y, cart.x));
    return angle;
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