#include "rectangle.h"
#include <iostream>
#include <cmath>

Rectangle::Rectangle(){
    width_ = 2;
    height_ = 4;
    setCentre(0,0);
}
Rectangle::Rectangle(double width, double height){
    width_ = width;
    height_ = height;
}
void Rectangle::setHeightWidth(double width, double height){
    width_ = width;
    height_ = height;
}

bool Rectangle::intersectionCheck(double pointX, double pointY){
    if (pointX <= width_ && pointY <= height_){
        std::cout<<"Rectangle Intersection"<<std::endl;
        return true;
    }
    return false;
}