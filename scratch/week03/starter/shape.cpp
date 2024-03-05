#include "shape.h"
#include <vector>

Shape::Shape() :
    description_("unknown"), centreX_(0),centreY_(0)
{

}

std::string Shape::getDescription(){
   return description_;
}

void Shape::setCentre(double x, double y){
    centreX_ = x;
    centreY_ = y;
}

void Shape::offsetCentre(double x, double y){
    centreX_ += x;
    centreY_ += y;
}

double Shape::getcentreX(){
    double x = centreX_;
    return x;
}

double Shape::getcentreY(){
    double y = centreY_;
    return y;
}