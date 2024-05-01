#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>

class Controller: public ControllerInterface
{
  public:
  /**
   * @brief Default constructor should set all attributes to a default value,
   * as well as assigning values to private/protected variables from the class header file.
  */
  Controller();
  ~Controller();

  /**
  @brief Run controller in reaching goals - non blocking call, unlocks mutexes and updates conditional variables to enable @sa reachGoals function()
  */
  void run(void);

  /**
  @brief Movement to Reach goal - execute control to reach goal, called in thread, so non-blocking
  */
  virtual void reachGoals(void) = 0;
  /**
  @brief Returns platform status (indicating if it is executing a series of goals or idle - waiting for goals)
  @return platform status
  */
  pfms::PlatformStatus status(void);

  /**
  @brief Sets goal and checks if platform can reach goal from current poisiton, the function will update internal variables associated with: @sa timeToGoal
  and @sa distanceToGoal
  @param[in] goals The vector of goal points (x,y,z)
  @return goals reachable
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
  @brief Retrieves value for distance to be travelled to reach current goal, updates as the platform moves to current goal
  @return distance to be travelled to goal [m]
  */
  double distanceToGoal(void);

  /**
  @brief Retrieves value for time to reach current goal, updates at the platform moves to current goal
  @return time to travel to goal [s]
  */
  double timeToGoal(void);

/**
  @brief Returns total distance travelled by platform at the current time
  @return total distance travelled since started [m]
  */
  double distanceTravelled(void);

  /**
  @brief Returns total time in motion by platform at the current time, time when stationary not included
  @return total time in motion since started [s]
  */
  double timeTravelled(void);
  /**
  @brief Returns platform type, either Ackerman or Quadcopter
  @return platform type
  */
  pfms::PlatformType getPlatformType(void);

  /**
  @brief Set tolerance for acceptable margin when reaching goal
  @return tolerance accepted [m]
  */
  bool setTolerance(double tolerance);

  /**
  @brief Returns current odometry information from platform
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  pfms::nav_msgs::Odometry getOdometry(void);

  /**
  @brief Getter for obstacles detected (only in SUPER mode), otherwise return empty vector
  @param goals
  @return centre of obstacles detected
  */
  std::vector<pfms::geometry_msgs::Point> getObstacles(void);

  /**
  @brief Updates time and distance travelled from start to current time for both platforms.
  @return distance and time have been updated
  */
  bool updateDistTime(double velocity);

protected:
  //constanct variables (do not change after being set initially)
  pfms::PlatformType platformType_; //stores plat form type (Ackerman or Quadcopter)
  pfms::PlatformStatus platformStatus_; //stores what status the platform is in (IDLE, RUNNING, TAKEOFF/LANDING)
  std::vector<pfms::geometry_msgs::Point> goals_; //private copy of goals to be used throughout program
  double goalTolerance_; //stores value of acceptable tolerance to reaching goals
  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; //pfms pipes connector pointer to communicate with ROS nad the platforms

  //Constantly Updating variables
  pfms::nav_msgs::Odometry currentOdo_; //stores current odometry of the platform
  pfms::nav_msgs::Odometry previousOdo_; //stores previous odometry for calculating distance travelled every iteration (for total distance travelled calculations)
  pfms::nav_msgs::Odometry estimatedGoalPose_; //for checkOriginToDestination function input and setting goals, the estimated goal pose is required to be used as the next origin pose
  //distance and time to current goal, constantly updated through reachGoals calling checkOriginToDestination
  double distanceToCurrentGoal_; 
  double timetoCurrentGoal_;
  //total distance and time travelled from start to current time, recalculated throughout reachGoals
  double distanceTravelled_;
  double timeTravelled_;

  double velocity_; //private data member for velocity of platform

  //Threading variables
  std::mutex mtxStart_; //mutex variable used for controlling access to any shared resources
  std::condition_variable cvStart_; //condition variable to wait until run() is called, then continue with the reachGoals() function as called in the thread
  std::vector<std::thread> threads_; //vector to store threads and to be able to terminate then in destructor
  std::atomic<bool> running_;  //bool to indicate the loop in reachGoals should still be running (when running_==true)
};

#endif // CONTROLLER_H
