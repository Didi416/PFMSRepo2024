#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

/*!
 *  \brief     SkidSteer Class
 *  \details
 *  Derived class of Controller, includes functions needing to be implemented from Controller class 
 * virtual functions due to differing programming requirements from Ackerman.
 *  \author    Dyandra Prins
 *  \date      2024-27-02
 */

class SkidSteer: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  /**
  @brief Default constructor should set all attributes to a default value, includes values for platform type and velocity settings
  as well as assigning values to private/protected variables from the Ackerman class header file.
  **/
  SkidSteer();
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

  /**
   * @brief Movement for SkidSteer to be called in @sa reachGoal - Takes rotating and moving values and runs pfmsConnectorPtr cmd to communicate with simulation
   * @param[in] i Number of repeats/sequence to increment and move robot (Husky)
   * @param[in] turnLR Takes value of angular velocity and rotates husky left or right
   * @param[in] moveFB Takes value of velocity and moves husky forward or backward 
   * 
  */
  void drive(unsigned long i, double turnLR, double moveFB);

private:
  double velocity_;
  double angularV_;
  double angularDisp_;
  double turningAngle_;
  
  unsigned long i_;
  double turnLR_;
  double moveFB_;
};

#endif // SKIDSTEER_H
