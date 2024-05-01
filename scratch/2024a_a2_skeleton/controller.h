#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>

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
  Run controller in reaching goals - non blocking call
  */
  void run(void);

  virtual void reachGoals(void) = 0;
  /**
  Retrurns platform status (indicating if it is executing a series of goals or idle - waiting for goals)
  @return platform status
  */
  pfms::PlatformStatus status(void);

  /**
  @brief Sets goal and checks if platform can reach goal from current poisiton, the function will update internal variables associated with: @sa timeToGoal
  and @sa distanceToGoal
  @param[in] goal The goal point (x,y)
  @return goal reachable
  */
  bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);

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
  double timeTravelled(void);
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

  /**
  Getter for obstacles detected (only in SUPER mode), otherwise return empty vector
  @param goals
  @return centre of obstacles detected
  */
  std::vector<pfms::geometry_msgs::Point> getObstacles(void);

  bool updateDistTime(double velocity);

protected:
  pfms::PlatformType platformType_;
  pfms::PlatformStatus platformStatus_;
  std::vector<pfms::geometry_msgs::Point> goals_;
  double goalTolerance_;
  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;

  int currentGoalNum_;
  pfms::nav_msgs::Odometry currentOdo_;
  pfms::nav_msgs::Odometry previousOdo_;
  pfms::nav_msgs::Odometry estimatedGoalPose_;
  double distanceToCurrentGoal_;
  double timetoCurrentGoal_;
  double distanceTravelled_;
  double timeTravelled_;
  double startToCurrentGoalDist_;
  double totalDistance_;
  double totalTime_;

  //Threading variables
  std::mutex mtx_;
  std::mutex mtxStart_;
  std::condition_variable cvStart_;
  std::vector<std::thread> threads_;
  std::atomic<bool> ready_;
  std::atomic<bool> running_;
};

#endif // CONTROLLER_H
