#include "laser.h"
#include <vector>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {

    int repeats =10;
    if(argc !=2){
        std::cout << " Not arguments given on command line." << std::endl;
        std::cout << " usage: " << argv[0] << " <repeats>" << std::endl;
        std::cout << " defaulting to " << repeats << " <repeats>" << std::endl;
    }
    else{
        repeats = atoi(argv[1]);
    }

    //! Created a pointer to a Pipe (This is consumer and
    // std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>();
    Laser laser;
    std::this_thread::sleep_for (std::chrono::seconds(1)); //Small delay befroe starting the loop

     for (unsigned int i=0;i<repeats;i++){
         pfms::sensor_msgs::LaserScan laserScan;
         bool OK = laser.getLaserScan(laserScan);
         if(OK){
             std::cout << i << " " << std::setprecision(14) <<
                      laserScan.time << " " << std::setprecision(6) << 
                      laserScan.angle_max << " " <<
                      laserScan.angle_min << " " <<
                      laserScan.angle_increment << " " <<
                      laserScan.range_max << " " <<
                      laserScan.range_min << " ";

            // Can you find the closest point in the scan?
            //
            // Hint: The closest point is the smallest range
            //
            // You will then ned to use the angle_min, angle_increment and the index of the closest point to calculate the angle of the closest point
            // You will need to use the std::vector<double> ranges to get the range of the closest point 
            // Can you print the angle and range of the closest point?
            // Can you convert this to a cartesian coordinate?

             std::cout << std::endl;
         }
          else{
             std::cout << "No laser data available" << std::endl;
         }
         std::this_thread::sleep_for (std::chrono::milliseconds(100));
     }

    return 0;
}
