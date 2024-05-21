#include "sample.h"


Sample::Sample()
    : Node("a3_skeleton") //Node name is "a3_skeleton"
{
    //Subscribing to laser, this will call the laserCallback function when a new message is published
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "orange/laserscan", 10, std::bind(&Sample::laserCallback,this,std::placeholders::_1));    

    // The service is of type std_srvs::srv::Trigger
    // The service is created with the name "detect"
    // The callback function is detect
    // Callback function is called when the service is called
    // Change the service type and callback function to match the requirements of assessment task
    // service_ = this->create_service<std_srvs::srv::Trigger>("detect", 
    //             std::bind(&Sample::detect,this,std::placeholders::_1, std::placeholders::_2));

    // We create as an example here a function that is called via a timer at a set interval
    // Create a timer that calls the timerCallback function every 500ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Sample::timerCallback, this));

    // Finally we create a thread that will run the function threadFunction
    // The thread could start before any data is recieved via callbacks as we have not called spin yet
    // So consider this when you write your code
    thread_ = new std::thread(&Sample::threadFunction, this);

}

Sample::~Sample()
{
    // We join the thread here to make sure it is finished before the object is destroyed
    // Thanksfull, we check ros::ok() in the threadFunction to make sure it terminates, otherwise
    // we would have a deadlock here as the thread would be waiting for threadFunction to finish
    thread_->join();
}


void Sample::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
   /**
   * This callback creates a new laserProcessingPtr object if it does not exist
   * or updates the scan in the existing object
   */
    //If we have not created the laserProcessingPtr object, create it
    if(laserProcessingPtr_ == nullptr){
        laserProcessingPtr_ = std::make_unique<LaserProcessing>(*msg);
    }
    else{
        laserProcessingPtr_->newScan(*msg);
    }
}

// void Sample::detect(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
//                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
// {
//     // Call the countObjectReadings function from the laserProcessingPtr object
//     // check if laserProcessingPtr_ is not null   
//     if(laserProcessingPtr_ == nullptr){
//         RCLCPP_ERROR_STREAM(this->get_logger(),"No laser data available");
//         res->success = false;
//         res->message = "ERROR No laser data available";
//         return;
//     }
//     unsigned int readings = laserProcessingPtr_->countObjectReadings();
//     RCLCPP_INFO_STREAM(this->get_logger(),"valid readings:" << readings);
//     res->success = true;
//     res->message = "Readings: " + std::to_string(readings);
// }

void Sample::timerCallback()
{
    //This is an example of logging, this will be called every 500ms
    RCLCPP_INFO(this->get_logger(), "Timer callback example");
}

void Sample::threadFunction()
{
    // This function runs in a separate thread of execution, it is started in the constructor
    // We can call any function from here, but we need to be careful with the shared resources
    // We need to make sure that we are thread safe
    // This function needs to terminate when the node is destroyed and we can do so by checking if the node is still alive
    while(rclcpp::ok()){
        //This function will run every second
         if(laserProcessingPtr_ != nullptr){
            unsigned int readings = laserProcessingPtr_->countObjectReadings(); // readings here is a local variable, so not dealing with threading
            RCLCPP_INFO_STREAM(this->get_logger(),"in thread valid readings:" << readings);
         }
        std::this_thread::sleep_for(std::chrono::seconds(1));// we sleep for 1 second as an example here
    }
    
}