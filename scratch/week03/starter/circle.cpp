#include "circle.h"
#include <cmath>
#include <iostream>

Circle::Circle(): // This constructor can call a constructor that requires a parameter and supply a default parameter
    Circle(0.0)
{
}

Circle::Circle(double radius):
    radius_(radius)
{
    description_="circle";
    setCentre(0,0);
}

void Circle::setRadius(double radius){
    radius_ = radius;
}

double Circle::getCentreDist(){
    double r = radius_;
    return r;
}

double Circle::getPerimeter(){
    return 2 * M_PI *radius_; 
}

double Circle::getArea(){
    return M_PI *(radius_*radius_); 
}

void Circle::setArea(double area){
    radius_= std::pow(area_/M_PI,0.5);
}
void Circle::setPerimeter(double perimeter){
    radius_= perimeter_/(2*M_PI);
}

bool Circle::intersectionCheck(double pointX, double pointY){
    // for (int i = 0; i < 10; i++){
    //     double angleB = i*2*M_1_PI/360;
    //     double xPC = getcentreX() + (getCentreDist() * cos(angleB));
    //     double nxPC = getcentreX() - (getCentreDist() * cos(angleB));
    //     double yPC = getcentreY() + (getCentreDist() * sin(angleB));
    //     double nyPC = getcentreY() - (getCentreDist() * sin(angleB));

    //     if (nxPC<=pointX<= xPC && nyPC <= pointY <= yPC){
    //         std::cout<<"Intersection"<<std::endl;
    //         break;
    //     }
    // }

    if ((pointX - getcentreX())*(pointX - getcentreX()) + (pointY - getcentreY())*(pointY - getcentreY()) <= getCentreDist()*getCentreDist()){
        std::cout<<"Intersection"<<std::endl;
        return true;
    }
    return false;
}
