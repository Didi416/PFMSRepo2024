#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

/*!
 *  \brief     Quadcopter Class
 *  \details
 *  Derived class of Controller, includes functions needing to be implemented from Controller class 
 *  virtual functions due to differing programming requirements from Ackerman.
 *  \author    Dyandra Prins
 *  \date      2024-01-05
 */

class Quadcopter: public Controller
{
public:
  /**
  @brief Default constructor should set all attributes to a default value, includes values for variables to be sent to the command pipes
  as well as assigning values to private/protected variables from the Quadcopter class header file.
  **/
  Quadcopter();
  ~Quadcopter();
  /**
  @brief Called in setGoal to checks whether the platform can travel between origin of the platform and goal
  @param[in] origin The origin pose of the platform, specified as odometry readings
  @param[in] goal The goal point (x,y) for the platform to reach
  @param[out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);

  /**
  @brief Movement to Reach goal - execute control to reach goal, called in thread, so non-blocking
  */
  void reachGoals(void);
  /**
  @brief Sends to commands throough pfmsConnector pointer to the ROS pipes to control Quadcopter
  */
  void fly(unsigned long i, double turnLR, double moveLR, double moveUD, double moveFB);
  /**
  @brief Calculates and updates distances and angles for  movement to reach goal
  @return Goal reachable (true = goal can be reached, false = cannot be reached)
  */
  bool navCalcs(pfms::geometry_msgs::Point goal);

private:
  double turnLR_; //!<private data member for turning left/right command
  double moveFB_; //!<private data member for moving forward/back command
  double moveLR_; //!<private data member for moving left/right command
  double moveUD_; //!<private data member for moving up/down command
  double target_angle_; //!<private data member for target angle to current goal
};

#endif // QUADCOPTER_H
