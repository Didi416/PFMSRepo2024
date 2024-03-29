#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"
#include <chrono>   // Includes the system clock
#include <iostream>

/*!
 *  \brief     Controller Class
 *  \details
 *  Abstact base class for Ackerman and SkidSteer, includes all functions from Controller Interface class, implementing functions with common programming
 * and including virtual functions for those needing to be implemented in derived classes due to differing programming requirements.
 *  \author    Dyandra Prins
 *  \date      2024-27-02
 */

class Controller: public ControllerInterface
{
public:
  //Default constructors should set all attributes to a default value
  /**
   * @brief Default constructor should set all attributes to a default value,
   * as well as assigning values to private/protected variables from the class header file.
  */
  Controller();
  ~Controller();

  //See controllerinterface.h for more information
    /**
  @brief Movement to Reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  @return goal reached (true - goal reached, false - goal abandoned : not reached)
  */
  virtual bool reachGoal(void) = 0;

  /**
  @brief Sets goal and checks if platform can reach goal from current poisiton, the function will update internal variables associated with: @sa timeToGoal
  and @sa distanceToGoal
  @param[in] goal The goal point (x,y)
  @return goal reachable
  */
  bool setGoal(pfms::geometry_msgs::Point goal);

  /**
  @brief Called in setGoal to checks whether the platform can travel between origin of the platform and goal
  Implemented in derived classes Ackerman and SkidSteer
  @param[in] origin The origin pose of the platform, specified as odometry readings
  @param[in] goal The goal point (x,y) for the platform to reach
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;

  /**
  Retrieves value for distance to be travelled to reach goal, updates as the platform moves to current goal
  @return distance to be travelled to goal [m]
  */
  double distanceToGoal(void);

  /**
  Retrieves value for time to reach goal, updates at the platform moves to current goal
  @return time to travel to goal [s]
  */
  double timeToGoal(void);

/**
  Returns total distance travelled by platform
  @return total distance travelled since started [m]
  */
  double distanceTravelled(void);

  /**
  Returns total time in motion by platform, time when stationary not included
  @return total time in motion since started [s]
  */
  double timeInMotion(void);
  /**
  Returns platform type
  @return PlatformType
  */
  pfms::PlatformType getPlatformType(void);

  /**
  Set tolerance for when reaching goal
  @return tolerance accepted [m]
  */
  bool setTolerance(double tolerance);

  /**
  Returns current odometry information
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  pfms::nav_msgs::Odometry getOdometry(void);

protected:
  pfms::PlatformType platformType_;
  pfms::geometry_msgs::Point goal_;
  pfms::nav_msgs::Odometry currentOdo_;
  double distanceToCurrentGoal_;
  double timetoCurrentGoal_;
  double totalDistance_;
  double totalTime_;
  double originalDistanceToCurrentGoal_;
  double originalTimetoCurrentGoal_;
  double goalTolerance_;
  pfms::nav_msgs::Odometry estimatedGoalPose_;
  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(); //initialises shared pointer to be used for entire program;
};

#endif // CONTROLLER_H
