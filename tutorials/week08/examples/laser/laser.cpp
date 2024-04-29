#include "laser.h"

Laser::Laser()
{
    pfmsConnectorPtr_ = std::make_unique<PfmsConnector>();
}

Laser::~Laser(){
}


bool Laser::getLaserScan(pfms::sensor_msgs::LaserScan& laserScan)
{
    bool OK = pfmsConnectorPtr_->read(laserScan_);
    if(OK){
        laserScan = laserScan_;
    }
    return OK;
}