#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"
#include <string>

// TASK:
//Modify the file rectangle [rectangle](./a2/rectangle.h) so it inherits from the base class of shape [shape](./a2/shape.h)
//Correct the missing access specifiers of base class [shape](./b2/shape.h) so that the immplementaion of the [`Rectangle` constructor](./a2/rectangle.cpp) can still access the required member varaible of `Shape` but have it restricted to users of the `Rectangle` class.

class Rectangle: public Shape
{


public:
//    TASK
//    Enable the Rectangle class to have a special member function that enables the Rectangle to on creation have width_ , height_ initialised 
//    with values supplied by user of the class. This function needs to set the description_ on creation of the class to be either square or 
//    rectangle depending on the width and heigth supplied by the user.
//    You will need to add the declaration of this member function in the [header of Rectangle class](./a2/rectangle.h) as
//    well as implement this function in [implementation file of Rectangle class](./a2/rectangle.cpp).

    Rectangle();
    Rectangle(double width, double height);
    /**
     * @brief Function that sets width and height
     * @param width in [m]
     * @param height in [m]
     */
    void setHeightWidth(double width, double height);

    // make getArea() a method that
    // 1) Base class Shape defines
    // 2) Needs to be in Rectangle
    // 3) makes Shape a pure virtual class
    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    double getArea (void);

private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?
    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
