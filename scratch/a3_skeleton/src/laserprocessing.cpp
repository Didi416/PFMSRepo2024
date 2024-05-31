#include "laserprocessing.h" //include header file
#include <algorithm>
#include <numeric>
#include <iostream>


LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan), objectReadings_(0) //initialise global variables upon object creation
{
}

unsigned int LaserProcessing::countObjectReadings(){
    std::unique_lock<std::mutex> lck(mtx); //secure data access to laserscan
    sensor_msgs::msg::LaserScan laserScan = laserScan_; //create local variable of laserscan
    lck.unlock(); //unlock access to data for other threads

    //initialise variables for counting laser readings
    unsigned int count=0;
    std::vector<double> x;std::vector<double> y;
    for (unsigned int i = 0; i < laserScan_.ranges.size(); i++) //iterate through laser scan, checking ranges
    {
        //if current laser point range is between the min and max range, and not inf or nan, add to object reading count (detected object)
        if ((laserScan_.ranges.at(i) > laserScan_.range_min) &&
            (laserScan_.ranges.at(i) < laserScan_.range_max) &&
            !isnan(laserScan_.ranges.at(i)) &&
            isfinite(laserScan_.ranges.at(i))){

            count++;
            objectReadingIndex_.push_back(i); //also add the position in the laser scan of the located object (for use later)
        }
    }

    objectReadings_=count; //store in global variable (new scan, new readings)
    return objectReadings_; //return number of object detected
}

bool LaserProcessing::detectLargeObstacle(){ //to detect for object larger than a cone in front of the laser
    countObjectReadings(); //collect object readings data and initialise variables
    unsigned int count = 0;

    for(unsigned int i=0; i<objectReadings_; i++){ // loop through for the number of detected object readings
        if (laserScan_.ranges.at(objectReadingIndex_.at(i)) < 10){ //check if range is less than 10m in front of laser
            count++; //increase count of very close objects
        }
    }

    if(count>50) {return true;} //if number of object readings within 10m of car is greater than 50, there is something blocking the path
    else {return false;}
}

unsigned int LaserProcessing::countSegments()
{
    //initialise variables
    unsigned int count = 0;
    geometry_msgs::msg::Point point1;
    geometry_msgs::msg::Point point2;
    //distance calculation variables
    double euDist;
    double dx;
    double dy;

    std::vector<int> currentSeg;
    double prevRangeIndex;
    obstacles_.clear(); //clear previous obstacles data
    // std::cout<<"Start Cone Detect \n";

    for (unsigned int i=0; i<laserScan_.ranges.size(); i++) {
        if (std::isfinite(laserScan_.ranges.at(i)) && laserScan_.ranges.at(i) < laserScan_.range_max) {
            // std::cout<<"Range value: "<<currentRange<<" Prev range value: "<<prevRange<<std::endl;
            if (count == 0){ //if no other obstacle points have been recorded
                count++;
                currentSeg.push_back(i); //mark the first point in the scan
            }
            else{
                //convert polar ranges to cartesian points
                point1 = polarToCart(prevRangeIndex);
                point2 = polarToCart(i);
                // calculate distance between them 
                dx = point1.x - point2.x;
                dy = point1.y - point2.y;
                euDist = std::hypot(dx,dy); 
                if (euDist < 0.3){ //if distance between points is less than 0.3, it is part of the same object
                    currentSeg.push_back(i); //add to current segment
                }
                else{ //if ponits are greater than 0.3m apart
                    count++;
                    if(currentSeg.size()<15){ //check if segmen if greater than 15 (size of cone segment)
                        obstacles_.push_back(currentSeg); //add as segment if vector the size of cone, otherwise, it is probably another obstacle, do not add
                    }
                    currentSeg.clear(); //clear vector for next segment
                }
            }
            prevRangeIndex = i; //in event that next valid range is not the immediate next iteration, record index position
        }
    }
    if(currentSeg.size() != 0){ //if there are still valid readings at the end of the range size (add as an object)
        count++;
        obstacles_.push_back(currentSeg);
        currentSeg.clear();
        // std::cout<<"EuDist: "<<euDist<<" dx:"<<point1.x<<" dy:"<<point1.y<<std::endl;
        // std::cout<<"Obs size: "<<obstacles_.size()<<std::endl;
    }
    return count; //return number of segments
}

std::vector<geometry_msgs::msg::Point> LaserProcessing::detectConeCentres(){
    //initialise temporary point variables
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint;
    std::vector<geometry_msgs::msg::Point> cones; //vector to stor cone centre positions
    //variables to check that cones are adequately spaced (not multiple cones on a larger object)
    double coneDist;
    double minConeDist = 2;
    bool addCone;

    countSegments(); //get data from laser scan (populate obstacles_)
    // std::cout<<"Obst size: "<<obstacles_.size()<<std::endl;
    for (size_t i=0; i<obstacles_.size(); i++){ //iterate through obstacles_ vector
        tempPoint = segmentToPoint(i); //convert segments to one points (rough centre of the cone)
        point.x = tempPoint.x;
        point.y = tempPoint.y;
        point.z = 0.0;
        addCone = true; //assume adding cone to vector
        for(size_t j=0; j<cones.size(); j++){ //iterate through current vector of cones
            coneDist = std::hypot((point.x - cones.at(j).x),(point.y - cones.at(j).y)); //calculate deistance between current cone and other cones
            if(coneDist < minConeDist){ //if cone too close to another cone
                addCone = false; //do not add cone to vector
                // std::cout<<"Don't add cone \n";
            }
        }
        if(addCone){
            cones.push_back(point); //add valid cones to vector
        }
    }
    // std::cout<<"Cone size: "<<cones.size()<<std::endl;
    return cones; //return vector of cone centres
}

std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> LaserProcessing::detectRoad(std::vector<geometry_msgs::msg::Point> points){
    //initialise temporary variables
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point pointPair;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> roadPairs;
    // distance calculation variables
    double dx;
    double dy;
    double euDist;
    float tolerance = 3.5; //road width tolerance (not exactly 8m)
    double roadWidth = 8; //roughly 8m
    //for determining if road pairs are on opposite sides of the track
    std::pair<std::string, std::string> side; 
    bool paired = false;
    for (size_t i=0; i<points.size(); i++){ //Sort into pairs, one cone on either side of the audi, calculated in cars frame
        point = points.at(i); //temporary point1

        for (size_t j=0; j<points.size(); j++){
            pointPair = points.at(j); //temporary point2

            if(std::abs(point.y) < 0.6){ //check if cone is straight ahead of car (could be either left or right on a corner)
                if(pointPair.y > 1){ //check if the other cone if definitely left or right and assign the cones accordingly
                    side.second = "left";
                    side.first = "right";
                }
                else if(pointPair.y < -1){
                    side.second = "right";
                    side.first = "left";
                }
                paired = true; //mark as paired
            }
            else if(point.y > 0){side.first = "left";} //otherwise check for definitely left or right
            else{side.first = "right";}
            if(!paired){ //if not paired already, repeat previous for next cone point
                if(std::abs(pointPair.y) < 0.6){
                    if(point.y > 1){
                        side.first = "left";
                        side.second = "right";
                    }
                    else if(point.y < -1){
                        side.first = "right";
                        side.second = "left";
                    }
                }
                else if(pointPair.y > 0){side.second = "left";} //check definitively left or right
                else{side.second = "right";}
            }
            else{paired = false;} //reset to false

            if(side.first == side.second){ //check for if cones are already paired or if they are one the same side of the road
                continue;
            }
            dx = point.x - pointPair.x; //calculate difference in x
            dy = point.y - pointPair.y; //calculate difference in y
            euDist = std::hypot(dx,dy); //calculate total difference in distance
            //check if pair is within the tolerance of roadwidth, and both cones are less than 13m from the car, and that the 
            //cones are roughly the same x distance from the car
            if (std::abs(roadWidth-euDist) < tolerance && std::hypot(point.x, point.y)<13 && std::hypot(pointPair.x, pointPair.y)<13 && 
                                                                                                        std::abs(point.x - pointPair.x) < 9){
                roadPairs.push_back(std::make_pair((point), (pointPair))); //push back as a pair of cones
                break; //go to next cone pair
            }
        }
    }
    return roadPairs; //retun pairs
}

void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){ //store new laser scane message
    std::unique_lock<std::mutex> lck(mtx); //secure data access
    laserScan_ = laserScan; //update global variable
}

geometry_msgs::msg::Point LaserProcessing::polarToCart(unsigned int index) //convert polar coordinates (angle and distance) to cartesian points
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index; //determine angle at index point
    float range = laserScan_.ranges.at(index);//determine range at index point
    geometry_msgs::msg::Point cart;
    cart.x = static_cast<double>(range*cos(angle)); //cartesian x value from cosine of angle
    cart.y = static_cast<double>(range*sin(angle)); //cartesian y value from sine of angle
    return cart; //return cartesian point
}

geometry_msgs::msg::Point LaserProcessing::segmentToPoint(int i){ //internal function to convert segments to single points

    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point tempPoint;
    float avgX = 0.0;
    float avgY = 0.0;
    
    for (size_t j=0; j<obstacles_.at(i).size(); j++){
        tempPoint = polarToCart(obstacles_.at(i).at(j)); //convert polar ranges to cartesian points
        avgX += tempPoint.x; //add up x values
        avgY += tempPoint.y; //add up y values
    }
    avgX = avgX/obstacles_.at(i).size(); //divide by size of segment to get average x value
    avgY = avgY/obstacles_.at(i).size(); //divide by size of segment to get average y value

    point.x = avgX;
    point.y = avgY;
    point.z = 0.0;
    return point; //return average centre point
}