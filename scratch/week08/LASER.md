Week 8 - Bonus Laser Work 
=========================
The code needs  **Pipes** library (version 3.0.4 or higher), [follow guide for update](https://github.com/41012/pfms-support/tree/main?tab=readme-ov-file#upgrades)

While developing code you do not need to run the simulator. However prior to running the code run the simulator `ros2 launch gazebo_tf a2.launch.py` in a terminal window. 


-------

Laser
===============================

The [Laser class](./starter/laser/laser.cpp) connects to the simulator and obtains laser data. We call it from the main and then also can interrogate the laser data. Can you find the closest point to the quad copter using the laser data.

**Hint**: The closest point is the smallest range

- You will need to use the `std::vector<double> ranges` which is in the `pfms::sensor_msgs::LaserScan` to get the range of the closest point 
- You will need to use the angle_min, angle_increment and the index of the closest point to calculate the angle of the closest point
- Can you print the angle and range of the closest point?
- Can you see it on the simulator (might be hard to see in polar coordinates)?
- Can you convert this to a Cartesian coordinate?
- Can you use `send(pfms::geometry_msgs::Goal goal)` via `PfmsConnector` to send the closest object coordinate to visualiser
  - When doing this, you could move this inside laser for testing.
