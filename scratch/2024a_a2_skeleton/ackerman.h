#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

class Ackerman: public Controller
{
public:
  /**
  @brief Default constructor should set all attributes to a default value, includes values for Max Steering Angle, Max Braking Torque, Default Throttle,
  as well as assigning values to private/protected variables from the Ackerman class header file.
  **/
  Ackerman();
  ~Ackerman();
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

  void reachGoals(void);

private:
  double MAX_BRAKE_TORQUE;
  double DEFAULT_THROTTLE;
  unsigned long i_;
  double brake_;
  double steering_;
  double throttle_;
  double velocity_;
};

#endif // ACKERMAN_H
