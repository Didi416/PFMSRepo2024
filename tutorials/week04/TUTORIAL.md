Week 4 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example.  If you have any questions about the material, please raise them in the tutorial session.

Today's work builds upon solutions in week 03 which are in starter code.

Inheritance  
------------------

We use [shapes folder in starter](./starter/shapes) , look at `Shapes` , `Rectangle`  , `Triangle`, `Circle`, what is there relationship. Which functions should be defined and implemented in `Shape`, which should be left to be implemented in `Rectangle`  , `Triangle`, `Circle`,. How do we achieve this?

Examine `ShapeProcessing` class, Implement the constructor and `checkIntersections` function. Details of expected functionality are in the header file.

The main  emulates a process similar to Battleship, have a look at it's inner working (there is no fancy gui, just the logic).

![](https://articulate-heroes.s3.amazonaws.com/uploads/rte/udxpzlbr_Storyline%20E-Learning%20Battleship%20Game%20Template.GIF)



Library Creation and Unit Testing 
-----------------

We use [shapes folder in starter](./starter/shapes) , look at the  [CMakeLists.txt](./starter/shapes/CMakeLists.txt) is generating two **shared** libraries, one called `shapes` and the other `shapeprocessing`

This enables us to unit test of the libraries, which is essential to ensure it performs correctly. We therefore introduce Unit Tests at this point, so you can start to design your own tests for your code.

In [utest.cpp](./starter/shapes/test/utest.cpp) we are checking our implementation of intercept for the Circle. The syntax of the test is `TEST (IntercectTest, Circle)` the `TEST` indicates it is a unit test `IntercectTest` is the suite name and the individual test is `Circle`. So this is a suite of intercept tests and we plan to do this on all shapes. Can you add `TEST (IntercectTest, Rectangle)` and `TEST (IntercectTest, Triangle)` using the supplied example?

The tests will be compiled at the same time as your code is compiled. You can run all tests with minimal reporting using `make test` or all tests individually from the build directory using `./test/utest`.

Add more tests for the area using a different suite `AreaTest`, you will now need to compare values, and as values are floats the unit test needs to call `ASSERT_NEAR` and example is ` ASSERT_NEAR(a,b,1e-5)` where `a` and `b` are compared to a precision of `1e-5`.

Finally, can you create a test for shape processing? Which method do we need to test and how? 

There is a very comprehensive guide called [Googletest Primer](https://github.com/google/googletest/blob/master/docs/primer.md) .

## Using Audi and PfmsConnector Library 

We use [basic_control folder in starter](./starter/basic_control) , look at the  [CMakeLists.txt](./starter/shapes/CMakeLists.txt) it is linking against  libraries, one called `audi` and the other `pfmsconnector`It is using `find_package(pipes)` to find the package that contains these libraries. They are installed and part of your system, like `vector` or `iostream` are.

The same code we have been provides shows how to connect to `Ackerman` platform , via a `shared_pointer` to `PfmsConnector`object. We can then use the `send` `read` methods to send commands `pfms::commands::Ackerman` or goal `pfms::geometry_msgs::Goal` that is visualised. In the commands the sequence number is important, repeated values are ignored (used as a safety measure). We also create an object of type `Audi` which allows us to use some functionality for computing steering values. 

If you want to look at the header files of the libraries, right click on them (for instance right click on `Ackerman` in the file and select `Go to Definition`.

We need to control the  `Ackerman` to reach a goal that is straight in front of it (10m away) . Discuss: How did I come to this conclusion? Where is the `Ackerman` and where is the `goal` relative to it?

First using the office chairs in our lab, work as a group (nominate a drive and an engine/break person). Devise a strategy on how you would control the platform to reach a goal. Recall that you have `steering`, `throttle` and `break`. How do you go about this, if you were driving a car:

- when do you hold the throttle down
- when do you stop holding throttle
- when do you press the break and how much

What are the inputs you need to be able to achieve this strategy?

![](https://media4.giphy.com/media/fxO7BYgqS4RuT4GNdF/giphy.gif?cid=ecf05e47rwaq60s6tq5v6c0qutebd4i9uj8trh5s9kytt4fp&ep=v1_gifs_related&rid=giphy.gif&ct=g)
