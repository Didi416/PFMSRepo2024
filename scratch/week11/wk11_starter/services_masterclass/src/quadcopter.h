#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

//We include messages types for quadcopter
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

//! UAV drone platform controller
class Quadcopter: public Controller
{
public:
  Quadcopter();

  ~Quadcopter();

  bool reachGoal(void);
  /**
   * Calculates the angle needed for the quadcopter to reach a goal.
   * @return Always true - quadcopter has no unreachable goals.
   */
  bool calcNewGoal(void);

  bool checkOriginToDestination(geometry_msgs::msg::Pose origin, geometry_msgs::msg::Point goal,
                                 double& distance, double& time,
                                 geometry_msgs::msg::Pose& estimatedGoalPose);

//  pfms::PlatformType getPlatformType(void);

private:

  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  //! Angle required for quadcopter to have a straight shot at the goal
  double target_angle_ = 0;
  bool liftoff_;

  const double TARGET_SPEED;
  const double TARGET_HEIGHT_TOLERANCE;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff_; 
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif // QUADCOPTER_H
