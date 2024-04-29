Week 10 Tutorial Questions
=========================

Symbolically link the `wk10_starter` folder supplied 
```bash
cd ~/ros2_ws/src 
ln –s <your_git_repo>/tutorials/week10/wk10_starter .
cd ~/ros2_ws/
```
Build the package using the `colcon build --symlink-install` command. The `--symlink-install`portion of the command expedites install.

We have two packages that we will use to investigate the use of topics. The first one `topics_masterclass` uses Laser data and the position of the Platform to compute closes point to platform. In the second example `quadcopter_control` we build on quiz 4 part a1 and port this code to ROS, also inserting a way to directly send goals to a `Quadcopter`.

This weeks code will not compile unless you complete the exercises. You can delete the symbolic link after class

``` bash
cd ~/ros2_ws/src 
rm wk10_starter
cd ~/ros2_ws/
rm -rf build install
colcon build --symlink-install
```

# A) Working with Laser Data

We will be modifying the `week10_laser` package in `topics_masterclass` folder and  can run our code we have been developing using `ros2 run week10_laser sample` . When running code, do so with simulation `ros2 launch gazebo_tf a2.launch.py` . 

See the notes in `PfmsSample` class to complete this exercise.

## TASK 1: Subscribing to Laser and Processing Data
We need to modify the code so we can subscribe to the laser data from this platform. To do so Modify PfmsSample:

* In PfmsSample constructor subscribe to laser data (`sensor_msgs::msg::LaserScan`) on topic `orange/laserscan` and use `laser_callback` as the callback function.
* Add the member variable for the subscriber to the PfmsSample  
* Add the callback function to the PfmsSample class

Then, implement your code that finds the closest point in `LaserProcessing::closestPoint()`. 

This function is being called from `process` function, which is attached to a thread of execution independent from the callbacks. This is typical arrangement when we need to combine data from two sources. Inside the function we print the location of the nearest obstacle in the laser scan by calling `closestPoint`.

You can run your ROS2 node via `ros2 run week10_laser sample`

## TASK 2: Point in Global Coordinates
We will now look at transfering the data into "world" coordinates. We need to take the position of the robot into consideration and attempt to publish the data in "world" refernce frame.

As we want to utilise both sources of data (robot position and the laser that is on the robot), we have `PfmsSample::process()` which we will use to combine both sources of data and then compute the position in global coordinates.

Find the closest point in global coordinates {x,y} and print on screen.

# B) Working with Controlling a Platform

We will use `week10_quad` package in `quadcopter_control` folder and use `rosrun week10_quad sample` to run our code. 

To run the simulator `roslaunch gazebo_tf uav_a3.launch gui:=true` if it complains about the launch file, complete a `git pull` from the `pfms-support` package.

TASK 3: Controlling the Quadcopter
-----------------------------------------------
We need to subscribe and publish to topics to control the quadcopter, and can do so i base/derived class `Controller` and `Quadcopeter` 

Consider:

* which topics do we need to send controls to the quadcopter? 
  * What are the topic names and data types?

* which topics do we need to subscribe to, in order to determine where we are? 
  * What are the topic names and data types?

We can also use a topic to  give the quadcopter a goal to go to and have already inserted the details required.

The syntax is, in the below what is the data type, can you use the CLI to find what it contains? What is the topic name

```c++
this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 5, std::bind(&Controller::setGoal,this,_1));
```

In order to accommodate controlling quadcopter we need to modify the Controller and Quadcopter class. 

Modify Controller:

* In Controller constructor subscribe to odometry
* Add the member variable for the subscriber to the Controller  
* Add the callback function to the Constructor class

Modify Quadcopter:

* In Quadcopter constructor create publishers to send commands (control and takeoff) 
* Add the member variable for the subscriber to the Controller  
* Create the message in the `sendCmd` method and publish it.

You can run your ROS2 node via `ros2 run week10_quad sample`

You can send goals directly to your Quadcopter code via 

```bash
ros2 topic pub -1 /drone/goal geometry_msgs/msg/Point "{x: 0, y: 5, z: 2}"
```


TASK 4: Subscribe to another topic
---------------------------

This is a strech goal for today, let's receive information directly from rviz into this node, receive a [ClickedPoint from RViz](https://answers.ros.org/question/69019/how-to-point-and-click-on-rviz-map-and-output-the-position/) into this node

* Find the topic name `/clicked_point` in the list of topics and determine its type
* Create a callback for this topic name and correct type
* Print out the sequence number and x , y coordinates using `RCLCPP_INFO_STREAM`


[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
[Polar to Cartesian]: https://www.mathsisfun.com/polar-cartesian-coordinates.html
