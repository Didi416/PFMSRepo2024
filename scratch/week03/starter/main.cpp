#include <iostream>
#include "circle.h"
#include "rectangle.h"
#include <cmath>
#include <vector>

using std::cout;
using std::endl;

//Write a program that accepts a point (x,y location) and displays if a shape intersects that point (the point is within the area of the shape)?
std::vector<Shape *> intersectingShapes(std::vector<Shape *> shapeVec, double pointX, double pointY){
    std::vector<Shape *> intersectingShapes;
    for(auto shape : shapeVec){
        if (shape->intersectionCheck(pointX, pointY)){
            intersectingShapes.push_back(shape);
        }
    }
    return intersectingShapes;
}

int main(){

    // Circle circ(4);
    // circ.intersectionCheck(3,2);

    // Rectangle rect(4,4);
    // rect.intersectionCheck(2,2);

    std::vector<Shape *> shapeVec;
    Circle circle1(3);
    Rectangle rectangle(4,5);
    shapeVec.push_back(&circle1);
    shapeVec.push_back(&rectangle);
    
    intersectingShapes(shapeVec, 2, 2);

}