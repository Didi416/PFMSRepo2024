#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

class Ackerman: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  Ackerman();

  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);
/**
  Getter for distance to be travelled to reach goal, updates as the platform moves to current goal
  @return distance to be travelled to goal [m]
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

  /**
  returns current odometry information
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  pfms::nav_msgs::Odometry getOdometry(void);

private:
  double STEERING_RATIO;
  double LOCK_TO_LOCK_REVS;
  double MAX_STEER_ANGLE;
  double WHEELBASE;
  double MAX_BRAKE_TORQUE;
  double DEFAULT_THROTTLE;

  double goalMidP_ [2];

  //Odo readings
  unsigned long i_;
  double brake_;
  double steering_;
  double throttle_;
};

#endif // ACKERMAN_H
