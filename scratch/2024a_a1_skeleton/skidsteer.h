#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

class SkidSteer: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  SkidSteer();
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);
  /**
  Getter for distance to be travelled to reach goal, updates at the platform moves to current goal
  @return distance to be travlled to goal [m]
  */
  double distanceToGoal(void);

  /**
  Getter for time to reach goal, updates at the platform moves to current goal
  @return time to travel to goal [s]
  */
  double timeToGoal(void);

  /**
  Reach reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  @return goal reached (true - goal reached, false - goal abandoned : not reached)
  */
  bool reachGoal(void);

  void drive(unsigned long i, double turnLR, double moveFB);

  /**
  returns current odometry information
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  pfms::nav_msgs::Odometry getOdometry(void);

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
