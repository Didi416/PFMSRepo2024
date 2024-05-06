#include "detection_node.h"

//using std::placeholders::_1;

Detection::Detection()
    : Node("quiz5")
{
    //Subscribing to laser
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "orange/laserscan", 10, std::bind(&Detection::laserCallback,this,std::placeholders::_1));    

    //! @todo Allow an incoming service on service name "detect_road_centre"
    //!  Syntax: 
    service1_ = this->create_service<std_srvs::srv::Trigger>("detect_road_centre", 
               std::bind(&Detection::detect,this,std::placeholders::_1, std::placeholders::_2));
    //!
    //!  where service_type is the service message type, service_name is the name of the service 
    //!  and ClassName is the name of the class where the function is defined
    //!  and functionName is the name of the function that will be called when the service is called
    //!  std::placeholders::_1 and std::placeholders::_2 are placeholders for the request and response (which are passed to the function)
    //!  unlike the subscriber, the service callback function should have two arguments, one for the request and one for the response

}

Detection::~Detection()
{

}


void Detection::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
   /**
   * This callback store laser data to be used in service call
   */

    std::unique_lock<std::mutex> lck(laserData_.mtx);
    laserData_.scan = *msg;
    sensor_msgs::msg::LaserScan scan = laserData_.scan;
    lck.unlock();

    //If we have not created the laserProcessingPtr object, create it
    if(laserProcessingPtr_ == nullptr){
        laserProcessingPtr_ = std::make_unique<LaserProcessing>(scan);
    }
    else{
        laserProcessingPtr_->newScan(scan);
    }

    unsigned int segments = laserProcessingPtr_->countSegments();
    RCLCPP_INFO_STREAM(get_logger(),"segments:" << segments);
}

//! @todo - TASK 5: Change the detect function so it can be a callback for the service, and matches your declaration in header file
void Detection::detect(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{

    //! @todo: TASK 5: In the detect service callback set the response message to "Road centre detected" 
    //! and the success flag to true

    // Set the response message to "Road centre detected"
    res->message = "Road centre detected";

    // Set the success flag to true
    res->success = true;
}

