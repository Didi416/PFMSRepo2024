#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan)
{
}

/**
   * @todo - Ex 1 : Find the closest point {x,y} to the robot using sensor_msgs::LaserScan
   *
   * On command line type 'ros2 interface show sensor_msgs/msg/LaserScan'
   * What are we provided in this message?
   * Do we have the information in this message to find the closest point?
   * What part of the message do we need to iterate over?
   * How do we convert from range data to {x,y} [this is known as polar to cartesian](https://www.mathsisfun.com/polar-cartesian-coordinates.html)
   * Where is time of this message stored?
   * Is the closest point identified the same as the one you see as closest on the stage simulator? Why is this the case?
   */

geometry_msgs::msg::Point LaserProcessing::closestPoint()
{
  geometry_msgs::msg::Point pt;

  return pt;
}


void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    laserScan_=laserScan;
}

