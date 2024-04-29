Week 10
=========================

Review last weeks class material on using Command Line Interface (CLI) as well as reviewing tutorials on [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and [writing a publisher and subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).

We have provided full ROS2 tutorial material on publishers and subscribers which is available in this weeks `wk10_start/topics` folder. Of the nodes supplied the ones that you should examine are

```bash
wk10_starter/topics/minimal_subscriber/
   lambda.cpp          (most basic example of subscriber)
   member_function.cpp (example of subscriber in a class)
wk10_starter/topics/minimal_publisher/
   lambda.cpp          (most basic example of publisher)
   member_function.cpp (example of publisher in a class)
```

When compiling your code you will receive a note:

```bash
'examples_rclcpp_minimal_subscriber' is in: /opt/ros/humble
'examples_rclcpp_minimal_publisher' is in: /opt/ros/humble
If a package in a merged underlay workspace is overridden and it installs headers, then all packages in the overlay must sort their include directories by workspace order. Failure to do so may result in build failures or undefined behavior at run time.
If the overridden package is used by another package in any underlay, then the overriding package in the overlay must be API and ABI compatible or undefined behavior at run time may occur.

If you understand the risks and want to override a package anyways, add the following to the command line:
	--allow-overriding examples_rclcpp_minimal_publisher examples_rclcpp_minimal_subscriber
```

This is because ROS already ships with the compiled examples, we are adding the source code for viewing.

The tutorial (as well as Assignment 3) requires the pfms_support packages to be part of your ROS system. You would have already made pfms_support part of your `ros2_ws` to be able to work on Assignment 1 and 2.

**[TUTORIAL](./TUTORIAL.md)**
