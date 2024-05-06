#ifndef LASER_H
#define LASER_H

#include "pfmsconnector.h"

class Laser
{
public:
  //Default constructors should set all attributes to a default value
  Laser();
  ~Laser();

  bool getLaserScan(pfms::sensor_msgs::LaserScan& laserScan);

private:
  std::unique_ptr<PfmsConnector> pfmsConnectorPtr_;
  pfms::sensor_msgs::LaserScan laserScan_;

};

#endif // LASER_H
