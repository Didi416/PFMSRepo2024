#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

class SkidSteer: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  SkidSteer();
  ~SkidSteer();

  bool reachGoal(void);

  bool setGoal(pfms::geometry_msgs::Point goal);

  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);

  pfms::nav_msgs::Odometry getOdometry(void);

private:
  bool computeSteering(double& steering,double& dist);

private:
  unsigned int ugvSeq_;
  const double maxVel_;
  const double maxOmega_;
  double prevD_;
  double deltaD_;

};

#endif // SKIDSTEER_H
