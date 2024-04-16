Quiz 4
======

Part A
------

**PREAMBLE**

We have been provided an implementation of the `Controller` and `Quadcopter` classes similar to Assignment 1. The functionality has been implemented across the base class `Controller` and derived class `Quadcopter`, while there are some errors in the code that prevent it from running correctly. 

While some of the functionality will help in your work towards Assignment 2, for this quiz we are moving the `Quadcopter` at `0.4 m/s` whereas in Assignment 1 that max speed is `1.0 m/s`.  The quiz  has been developed against **Pipes** debian package ( version 3.0.4 or above ).  You will receive an error if you do not have this version and can refer to [pfms-support](https://github.com/41012/pfms-support/?tab=readme-ov-file#upgrades) for how to upgrade.

We have been provided some tests to let us debug, isolate and fix issues in Quadcopter. The unit tests are in [`utest.cpp` in ./a1/test folder](./a1/test/utest.cpp)` and they can be run from build directory via `./test/utest` and you can also find them in Qtcreator (select `utest to run). There are three tests provided in the `QuadcopterTest` suite, for TASK 1-TASK 3.  Before running the unit tests make sure you have run the simulator `ros2 launch gazebo_tf a2.launch.py`.  If your having issues with the `Quadcopter` lifting of the ground try `ros2 topic pub -1 /drone/takeoff std_msgs/msg/Empty` in terminal. 

**TASK 1 - Initialisation of variables**

All member variables belonging to the class need to be initialised to default values. When we have a base and derived class the initialisation can occur in either the [Base Class Constructor (Controller)](./a1/controller.cpp)  or the [Derived Class Constructor (Quadcopter)](./a1/quadcopter.cpp) .  Initialise the variables in the appropriate constructor so that the `Constructor` tests pass. 

HINT: Check what is being tested in `Contrsuctor` test, look where the variables exist and consider where they should be initialised. Consider that you would also have another derived class `Ackerman` in Assignment 2.  

**TASK 2 - Reaching Goal function**

The function  [reachGoal in Quadcopter](./a1/quadcopter.cpp) is not implemented completely. We need to compute the control of total velocity `TARGET_SPEED`, which should be divided into the left/right and forward/backward velocity. To compute how we control the platform we can use the orientation of the platform `odo_.yaw` and the `target_angle_` which tells us where we should be heading. Currently when running 

Compute the correct control value and send the correct command. This will enable completion of `Goals` test which visits two goals.

At present the `reachGoal` will terminate, if we have not reached the goal within 2s of the estimated time. This prevents the code from running infinitely and is a wise move (make a bound by which your code should stop execution).

**TASK 3 - Compute time travelled**

The code should update the total time travelled `time_travelled_` it fails to do this currently. Look at how we compute whether to abort `reachGoal ` as you could use the same function for this task. Also, consider, if we aborted the function `reachGoal`, should we update the time travelled? 

**TASK 4 - Compute distance travelled**

While the estimated distance at the beginning is a rough guide, as the platform moves it could travel more than anticipated (estimated), especially if it  was going against wind (or in case of Ackerman if it was drifting). A better way to computer the distance travelled is increment `distance_travelled_` as you go along.  Also, consider, if we aborted the function `reachGoal`, should we update the distance travelled thus far?  


Part B
-------

To undertake testing and developing your code you only need to add symbolically link quiz4 part a2 to your ros2_ws. 

For instance my git is ~/git/pfms-2024a-alalemp (change for your location of your git) and therefore only once do i need to complete below.

```bash
cd ~/ros2_ws/src
ln -s ~/git/pfms-2024a-alalemp/quizzes/quiz4/a2 
```

Open up the code by opening the `ros2_ws/src` folder in vscode. You should be able to see an `a2` folder and we need to implement a single function in  [analysis](./a2/src/analysis.h) class.

When you need to compile your code you need to be in folder `~/ros2_ws/`and compile via a`colcon` command, you need to execute the command every time you change the code and want to execute the code with the changes. This can not be done via vscode.

You can either build all packages in `ros2_ws` via `colcon build --symlink-install`or you can specify a single package `colcon build --symlink-install --packages-select quiz4_a2`for instance.

To check the unit test, in the terminal you run

```bash
ros2 run quiz4_a2 utest
```

**TASK 5 - Count characters**

Counts the number of characters (including spaces, special characters and numbers) in the string supplied: for instance "foo boo 12" has 9 characters. 

