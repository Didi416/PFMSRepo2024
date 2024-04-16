#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

class Quadcopter: public Controller
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter();
  ~Quadcopter();
  /**
  @brief Called in setGoal to checks whether the platform can travel between origin of the platform and goal
  @param[in] origin The origin pose of the platform, specified as odometry readings
  @param[in] goal The goal point (x,y) for the platform to reach
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);

  /**
  Run controller in reaching goals - non blocking call
  */
  void run(void);
  void reachGoals(void);
  void fly(unsigned long i, double turnLR, double moveLR, double moveUD, double moveFB);

private:
  double velocity_;
  double turnLR_;
  double moveFB_;
  double moveLR_;
  double moveUD_;
};

#endif // QUADCOPTER_H
