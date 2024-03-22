#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"
#include <chrono>   // Includes the system clock
#include <iostream>

class Controller: public ControllerInterface
{
public:
  //Default constructors should set all attributes to a default value
  Controller();
  ~Controller();

  //See controllerinterface.h for more information
    /**
  Reach reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  @return goal reached (true - goal reached, false - goal abandoned : not reached)
  */
  virtual bool reachGoal(void) = 0;

  /**
  Setter for goal, the function will update internal variables asscoiated with @sa timeToGoal
  and @sa distanceToGoal
  @return goal reachable
  */
  bool setGoal(pfms::geometry_msgs::Point goal);

  /**
  Checks whether the platform can travel between origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
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
  Getter for distance to be travelled to reach goal, updates as the platform moves to current goal
  @return distance to be travlled to goal [m]
  */
  virtual double distanceToGoal(void) = 0;

  /**
  Getter for time to reach goal, updates at the platform moves to current goal
  @return time to travel to goal [s]
  */
  virtual double timeToGoal(void) = 0;

/**
  returns total distance travelled by platform
  @return total distance travelled since started [m]
  */
  double distanceTravelled(void);

  /**
  returns total time in motion by platform, time when stationary not included
  @return total time in motion since started [s]
  */
  double timeInMotion(void);
  /**
  Getter for pltform type
  @return PlatformType
  */
  pfms::PlatformType getPlatformType(void);

  /**
  Set tolerance when reaching goal
  @return tolerance accepted [m]
  */
  bool setTolerance(double tolerance);

  /**
  returns current odometry information
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  virtual pfms::nav_msgs::Odometry getOdometry(void) = 0;

protected:
  pfms::PlatformType platformType_;
  pfms::geometry_msgs::Point goal_;
  pfms::nav_msgs::Odometry currentOdo_;
  double distanceToCurrentGoal_;
  double timetoCurrentGoal_;
  double totalDistance_;
  double totalTime_;
  
  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;
};

#endif // CONTROLLER_H
