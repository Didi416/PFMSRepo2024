Week 9 Tutorial Questions
=========================

ROS2 is a middleware, each executable is also called a node, and communicates with other nodes via topics and services (topics - one way communication like a produce/consumer; services - two way communication with a guaranteed return value).

One of the larger changes between ROS1 and ROS2 is the way the nodes establish communication. In ROS1 this was done via a central server called `roscore`. ROS2 does not have a central server, rather the nodes use `avahi` to discover topics and services.

You need to look at the canvas pre-work for week 9:

1. Shows [using turtlesim, ros2, and rqt](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

2. Examine [looking at nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

3. [Understanding topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

4. [Understanding services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

5. ROS [logging mechanism](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)

Now we can use this knowledge to explore the ROS2 ecosystem used for Assessment 2 simulator. 

Exploring the Command Line Interface (CLI)
----------------------------

Unlike the tutorial on canvas thus far, instead of runnign one node at a time we use `launch` to stat several `nodes` at once.

```bash
ros2 launch gazebo_tf a2.launch.py
```

The simulator has two robots (Ackerman and Quadcopter) equipped with sensors (Laser and a Sonar Array -number of Sonars).  All ROS2 CLI command contain `ros2` as the first argument, as noted above, refer [CANVAS Week 09 Prework](https://canvas.uts.edu.au/courses/30581/pages/week-09-prework) if your uncertain about the commands

### Questions And Tasks ###

* What executables are part of the `gazebo_tf` package?

* What nodes are running?

* Access more information `info` about the `/audi_reach` node?

* Access more information `info` about the `/drone/control` node?

* What are the topics available?

* Name two differences between the `/drone/gt_odom` and  `/drone/laserscan` topics?

* Print the contents on the `/orange/odom` topic to screen using `ros2 topic echo <topic_name>`

* What are the fields in this message? HINT use `ros2 interface show <msg_name>`

* What message type is being sent on  `/drone/cmd_vel`? (HINT: use `ros2 topic info` and `ros2 interface show`

* What message type is being sent on  `/drone/takeoff`? (HINT: use `ros2 topic info` and `ros2 interface show`

* Send a takeoff message. HINT use `ros2 topic pub -1 <topic name> <message>` to send one mesage to the topic.

* Send a linear velocity of 0.1 m/s to go up via the `/drone/cmd_vel`  using `rostopic pub` (HINT `ros2 topic pub -h` can tell you how to send data at a specific rate, the below would send all zeros 10 times.)

  ```bash
  ros2 topic pub -r 10 /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
  ```
  
* List all ROS services available

* Let's use `/demo/get_entity_state` , what does the below show?

```bash
ros2 service call /demo/get_entity_state gazebo_msgs/srv/GetEntityState '{name: audibot, reference_frame: world}'
```

* Let's move the orange audi and drone and reset world

Moving orange audibot

  ```
  ros2 service call /demo/set_entity_state 'gazebo_msgs/srv/SetEntityState' '{state: {name: "orange", pose: {position: {x: -2, y: 1}}}}'
  ```

Moving drone

  ```
   ros2 service call /demo/set_entity_state 'gazebo_msgs/srv/SetEntityState' '{state: {name: "drone", pose: {position: {x: 5, y: 5}}}}'
  ```

Reset world

`ros2 service call /reset_world std_srvs/srv/Empty`

* Reset the world and move audibot and drone 5m above the ground (keep the x and y the same)? 
  * How do you get current position?
  * How do you send the place you want them to start from?
* Takeoff the drone, then move it 5m above the ground and keep it's x,y the same.
* Move the audi to be facing `pi/2` in terms of yaw
  * What is the orientation specific as?
  * How to convert yaw to quaternion?

* Use rqt_console to view the log from `audi_reach`, can you change the default log level?

  
