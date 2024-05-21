#include "mission.h"
#include <iostream>

// void Mission::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
// {

//     //! On each callback will save message to file, space seperated x y z values (row for each message)
//     geometry_msgs::msg::Point pt = msg->pose.position;


//     visualization_msgs::msg::Marker marker = produceMarker(pt);
//     markerArray_.markers.push_back(marker);

//     pub_->publish(markerArray_);

//     //Let's also send the goal to quacdopter

//     goals_.push_back(pt);

//     RCLCPP_INFO_STREAM(this->get_logger(),"Have: " << goals_.size() << " goals");

//     //pub_goals_->publish(pt);
// }