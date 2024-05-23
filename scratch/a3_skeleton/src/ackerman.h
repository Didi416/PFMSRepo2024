#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

/*!
 *  \brief     Ackerman Class
 *  \details
 *  Derived class of Controller, includes functions needing to be implemented from Controller class 
 *  virtual functions due to differing programming requirements from Quadcopter.
 *  \author    Dyandra Prins
 *  \date      2024-01-05
 */

class Ackerman: public Controller
{
public:
  /**
  @brief Default constructor should set all attributes to a default value, includes values for Max Steering Angle, Max Braking Torque, Default Throttle,
  as well as assigning values to private/protected variables from the Ackerman class header file.
  **/
  Ackerman();
  ~Ackerman();
  /**
  @brief Movement to Reach goal - execute control to reach goal, called in thread, so non-blocking
  */
  void reachGoals(void);

  void timerCallback();

  /*! @brief - A function that will be run in a separate thread
  * We will simply do some logging here in this function as an example
  */
  void threadFunction();

private:

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brakePub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steeringPub_; 
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttlePub_; 

  double MAX_BRAKE_TORQUE; //maximum brake torque, set to 8000Nm
  std_msgs::msg::Float64 brake_; //private data value for braking command
  std_msgs::msg::Float64 steering_; //private data member for steering command
  std_msgs::msg::Float64 throttle_; //private data member for throttle command
  double velocity_;
  double tolerance_;
  std::thread* thread_; //!< Thread object pointer
};

#endif // ACKERMAN_H