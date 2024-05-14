Week 11 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. 
If you have any questions about the material, please raise them in the next lab session.

This week's lab questions will make use of pfms-support provided wiith assignment 1/2, do a git pull from the pfms-support folder.

Before you get started, make sure you do the following:

Link the `wk11_starter` folder to your ros2_ws workspace folder, (ie if your path is <YOURGIT>/tutorial/week11/wk11_starter then execute:

```bash
cd ~/ros2_ws/src
ln -s <YOURGIT>/tutorial/week11/wk11_starter
```

Compile packages using `colcon` 

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

The very first time your build a package it get's added to ROS and therefore you need to `sources ~/.bashrc` only once (or from within `~/ros2_ws` you can do `source install/setup.bash` which has same effect).

Focus of exercises is using services and topics.

## TASK 1 - Client and Service Node

Firstly there is an example of using services in two way communication where we have both `service node` and  `client node`. This was based on [example from foxglove](https://foxglove.dev/blog/creating-ros2-services). In the example we have two nodes that need to communicate. Unlike topics, which follow a publish-subscribe model to send each other information, services follow a request-response model.

With services, a client node must first **call** the server in order to send a **request**. This means that in the server-client mode, nodes do not use a  communication stream until it’s absolutely needed, preventing  unnecessary bandwidth usage. Then, the client node must wait for a **response** from the server (or until a timeout is reached). For this reason, services can confirm communication between nodes.

![](https://foxglove.dev/images/blog/creating-ros2-services/hero.webp)

Examine the files, look at the narrative [from Foxglove dev's](https://foxglove.dev/blog/creating-ros2-services). To run code in two terminals run

```bash
ros2 run srv_client_checks diagnostics_node
```

And

```bash
ros2 run srv_client_checks motor_node
```

Open the files and answer following questions

- What is the name of the package?
- What dependencies does it have?
- How many executables are there?
- What are the names of the nodes?
- How many topics is each node listening to?
- What is the server and what client node
- Look at syntax for client/server, what are they similar to?
- What does std:::bind do?
- What is the other different between the code for the two nodes?
- What is different between our supplied solution and that of foxglove or ROS webiste?

TASK 2: Using ROS Service
----------------------------

Our Quadcopter will need to receive a oal via topic name `/drone/goal` of the type `geometry_msgs/msg/Point`.  We need to accomodfate a incoming service call (we are a service node) that can be made on `/reach_goal`  of type `std_srvs/srv/SetBool` which runs function `Quadcopter::request`. To view the service message type `ros2 interface info std_srvs/srv/SetBool`. We aim to track the status of the quadcopter via `quadcopter::PlatformStatus status_` , status is defined in `quadcopter.h` and return information via return fileds.

We can run in two separate terminals

```bash
ros2 launch gazebo_tf a3_audi_drone.launch
ros2 run week11_quad sample
```

To send a goal via the ROS2 CLI.

```bash
 ros2 topic pub -1 /drone/goal geometry_msgs/msg/Point "{x: 0, y: 5, z: 2}"
```

**Modify the code to incorporate the service call**

We have selected the `std_srvs` package and the `SetBool` service name.

Use `ros2 interface show /std_srvs/srv/SetBool`  to examine this service type. 

This means we now need to let our package `quiz5` know we need the `std_srvs` package as a dependency. Then we need to create a service object, tie it to a service name and have a callback function. 

For this exercise, we will not complete anything sophisticated. In callback function you need to augment the field so the Response to the service call changes values - as described below in full steps. 

**Steps**

[package.xml](./wk11_starter/services_masterclass/package.xml) 

- [ ] Add the `std_srvs` package as both `build_depend` and `exec_depend`

[CMakeLists.txt](./wk11_starter/services_masterclass/CMakeLists.txt) 

- [ ] Add a  `find_package` for `std_srvs`. 
- [ ] As our library `controller` needs to use the `std_srvs` package/library we need to add to the existing `ament_target_dependencies` of `controller` library, we need to add the `std_srvs` library.

[controller.h](./wk11_starter/services_masterclass/src/cotroller.h) 

- [ ] Include the SetBool message

  `Syntax: #include "package_name/srv/service_name.hpp"`

- [ ] Create a service object

  `Syntax: rclcpp::Service<service_type>::SharedPtr variable_name_;`

- [ ] Create a `control` function so it can be a callback for the service

​	` Syntax: void function_name(const std::shared_ptr<service_type::Request>  req,std::shared_ptr<service_type::Response> res);`

[controller.cpp](./wk11_starter/services_masterclass/src/controller.cpp) 

- [ ] Create a service object of service type `std_srvs/srv/SetBool` on service name `reach_goal` with function name `control` as callback.
- [ ] Immpelemt the `control` function as per below.
- [ ] Change the Request and Response fields as per below


**Modify the code so that**

1. We need to track the status of Quadcopter, include a platform status `pfms::PlatformStatus` that tracks current status of Quadcopter.

2. Enable to query and change the ststus via the service call  (which activates `control` function) and the data field in request portion 
   *  if data is true
     * will `TAKEOFF` if the platform is `IDLE` and there are no goals
     * will `TAKEOFF` and then perform execution of the goal (go into `RUNNING`) if there is a goal set that is not yet reached
     * will keep in `RUNNING` is still going to a goal
   * if data is false
     * will perform `LANDING` and then go to `IDLE` irrespective if there is a goal

3. In rResponce we can obtain the current status of the Quadcopter via the service and the success and message part of the response portion

* The `success` field
  * returns true if there is a goal set that is not yet reached
* The `message` field 
  * shall have a string corresponding to current status (for instance `IDLE` status responds back with `IDLE` string value)

**HINTS**

* There is a thread of execution `reachGoal` lineked to a timer. Keep this the loop that runs through at 10Hz  (play around with the rate needed)
* Check your select/case or if/then statements to handle status 
* How if the odometry accesses and why is it private in controller.h? 
* We have had to place the PlatformStatus value under namespace quadcopter
* Do we have to assume quadcopter is `IDLE` at startup ... or can we check the odometry?


[services_masterclass]: starter/services_masterclass
[utest.cpp]: starter/services_masterclass/test/utest.cpp
[quiz5a]: ../../quizzes/quiz5/a
[pfms_support]: ../../skeleton/pfms_support
