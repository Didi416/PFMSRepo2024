#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

/*!
 *  \brief     Ackerman Class
 *  \details
 *  Derived class of Controller, includes functions needing to be implemented from Controller class 
 *  virtual functions due to differing programming requirements from SkidSteer.
 *  \author    Dyandra Prins
 *  \date      2024-27-02
 */

class Ackerman: public Controller
{
public:
  /**
  @brief Default constructor should set all attributes to a default value, includes values for Max Steering Angle, Max Braking Torque, Default Throttle,
  as well as assigning values to private/protected variables from the Ackerman class header file.
  **/
  Ackerman();
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
  Movement to Reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  @return Goal reached (true - goal reached, false - goal abandoned/not reached)
  */
  bool reachGoal(void);

private:
  double WHEELBASE;
  double MAX_BRAKE_TORQUE;
  double DEFAULT_THROTTLE;

  //Odo readings
  unsigned long i_;
  double brake_;
  double steering_;
  double throttle_;

};

#endif // ACKERMAN_H
