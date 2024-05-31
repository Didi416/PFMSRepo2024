#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

/*!
 *  \brief     Ackerman Class
 *  \details
 *  Derived class of Controller, includes functions needing to be implemented from Controller class  and thread functions, aswell as relevant publishers and data variables
 *  \author    Dyandra Prins
 *  \date      2024-31-05
 */

class Ackerman: public Controller
{
public:
  /**
  @brief Default constructor should set all attributes to a default value, includes values for Max Steering Angle, Max Braking Torque
  as well as assigning values to private/protected variables from the Ackerman class header file.
  **/
  Ackerman();

  /*!
  @brief Ackerman destructor, tears down object
  */
  ~Ackerman();

  /**
  @brief Movement to Reach goal - execute control to reach goal, called in thread, so non-blocking
  */
  void reachGoals(void);

  /*! @brief - A function that will be run in a separate thread, in this case the reachGoals function
  */
  void threadFunction();

private:

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brakePub_; //!< brake control publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steeringPub_; //!< steering control publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttlePub_; //!< throttle control publisher

  Audi audi; //!< audi library object for calculation of steering and distance to goals
  double MAX_BRAKE_TORQUE; //!< maximum brake torque, set to 8000Nm
  std_msgs::msg::Float64 brake_; //!< private data value for braking command
  std_msgs::msg::Float64 steering_; //!< private data member for steering command
  std_msgs::msg::Float64 throttle_; //!< private data member for throttle command
};

#endif // ACKERMAN_H