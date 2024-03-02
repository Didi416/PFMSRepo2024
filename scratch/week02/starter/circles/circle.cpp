#include <iostream>
#include <cmath>
#include "circle.h"

const double pi = 3.14159;

Circle::Circle(int r){
    _radius = r;
    std::cout << "Circle with radius " << _radius << " is created." << std::endl;
}

void Circle::setRadius(int r){
    _radius = r;
    std::cout << "Circle now has radius " << _radius << "." << std::endl;
}

double Circle::returnArea(void){
    auto area = pi * _radius*_radius;
    //std::cout << "Circle with radius " << _radius << " has area of " << area << "." << std::endl;
    return area;
}

double Circle::returnPerimeter(void){
    auto per = 2*pi * _radius;
    return per;
}

void Circle::setArea(int area){
    _radius = sqrt(area/(pi));
    std::cout << "Set circle's area to " << area << " and radius is now " << _radius << "." << std::endl;
}

void Circle::setPerimeter(int per){
    _radius = per/(2*pi);
    std::cout << "Set circle's perimeter to " << per << " and radius is now " << _radius << "." << std::endl;
}