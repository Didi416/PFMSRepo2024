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

bool LaserProcessing::detectLargeObstacle(){
    countObjectReadings();
    unsigned int count = 0;

    for(unsigned int i=0; i<objectReadings_; i++){
        if (laserScan_.ranges.at(objectReadingIndex_.at(i)) < 10){
            count++;
        }
    }

    if(count>50) {return true;}
    else {return false;}
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
    double prevRangeIndex;
    obstacles_.clear();
    // std::cout<<"Start Cone Detect \n";

    for (unsigned int i=0; i<laserScan_.ranges.size(); i++) {
        if (std::isfinite(laserScan_.ranges.at(i)) && laserScan_.ranges.at(i) < laserScan_.range_max) {
            // std::cout<<"Range value: "<<currentRange<<" Prev range value: "<<prevRange<<std::endl;
            if (count == 0){ //if no other obstacle points have been recorded
                count++;
                currentSeg.push_back(i);
            }
            else{

                point1 = polarToCart(prevRangeIndex);
                point2 = polarToCart(i);
                dx = point1.x - point2.x;
                dy = point1.y - point2.y;

                euDist = std::hypot(dx,dy);
                if (euDist < 0.2){
                    currentSeg.push_back(i);
                    // std::cout<<"EuDist: "<<euDist<<" Seg size: "<<currentSeg.size()<<std::endl;
                }

                else{
                    count++;
                    obstacles_.push_back(currentSeg);
                    currentSeg.clear();
                    // std::cout<<"EuDist: "<<euDist<<" dx:"<<point1.x<<" dy:"<<point1.y<<std::endl;
                    // std::cout<<"Obs size: "<<obstacles_.size()<<std::endl;
                }
            }
            prevRangeIndex = i;
        }
    }
    if(currentSeg.size() != 0){
        count++;
        obstacles_.push_back(currentSeg);
        currentSeg.clear();
        // std::cout<<"EuDist: "<<euDist<<" dx:"<<point1.x<<" dy:"<<point1.y<<std::endl;
        // std::cout<<"Obs size: "<<obstacles_.size()<<std::endl;
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
    // std::cout<<"Obst size: "<<obstacles_.size()<<std::endl;
    for (size_t i=0; i<obstacles_.size(); i++){
        tempPoint = segmentToPoint(i);

        point.x = tempPoint.x;
        point.y = tempPoint.y;
        point.z = 0.0;
        addCone = true;
        for(size_t j=0; j<cones.size(); j++){
            coneDist = std::hypot((point.x - cones.at(j).x),(point.y - cones.at(j).y));
            if(coneDist < minConeDist){
                addCone = false;
                // std::cout<<"Don't add cone \n";
            }
        }
        if(addCone){
            cones.push_back(point);
        }
    }
    // std::cout<<"Cone size: "<<cones.size()<<std::endl;
    return cones;
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