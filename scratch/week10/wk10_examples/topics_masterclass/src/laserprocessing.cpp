#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan)
{
}

//! @todo
//! TASK 1 - Refer to Header file for full description
geometry_msgs::msg::Point LaserProcessing::closestPoint()
{
  geometry_msgs::msg::Point pt;

  float nearest_range = laserScan_.range_max;
  unsigned int nearest_index = 0;
  for (unsigned int i=0; i<laserScan_.ranges.size(); i++) {
      if (laserScan_.ranges.at(i) < nearest_range) {
          nearest_range = laserScan_.ranges.at(i);
          nearest_index = i;
      }
  }

  float nearest_angle = laserScan_.angle_min + nearest_index * laserScan_.angle_increment;

  pt.x = nearest_range * cos(nearest_angle);
  pt.y = nearest_range * sin(nearest_angle);
  pt.z = 0;
  return pt;
}


void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    laserScan_=laserScan;
}

