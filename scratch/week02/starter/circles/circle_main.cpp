// We need to include the declaration of our new circle class in order to use it.
#include "circle.h"
#include <iostream>
#include <vector>
#include <random>

double returnAreaOfCirclesVector(std::vector<Circle> circleVec){
    double totalArea;
    for (int i=0; i<circleVec.size(); i++){
        double indivArea = circleVec.at(i).returnArea();
        //std::cout<<indivArea<<std::endl;
        totalArea += indivArea;
    }
    return totalArea;
}

int main () {
    //Ex01 - (Uncomment below to line 25)
    // Circle circle(4);
    // std::cout << "Circle has area of " << circle.returnArea() << "." << std::endl;
    // std::cout << "Circle has perimeter of " << circle.returnPerimeter() << "." << std::endl;
    // circle.setRadius(3);
    // circle.setArea(8);
    // circle.setPerimeter(9);
    // return 0;

    //Ex02 - (Uncomment below to line 39)
    // std::vector<Circle> circleVec;
    // Circle C1(1.0);
    // Circle C2(2.0);
    // Circle C3(5.0);s
    // circleVec.push_back(C1);
    // circleVec.push_back(C2);
    // circleVec.push_back(C3);

    // for (int i=0; i<3; i++){
    //     std::cout<<"Area of C"<<i+1<<" is "<<circleVec.at(i).returnArea()<<"."<<std::endl;
    //     std::cout<<"Perimeter of C"<< i+1<<" is "<<circleVec.at(i).returnPerimeter()<<"."<<std::endl;
    // }

    //Ex03
    // Function that accepts a vector of circles and returns the combined area of all circles in the vector.

    // Uncomment below for Ex03, without randomiser
    // int num_circles;
    // double circle_radius;
    // std::vector<Circle> circleVec2;
    // std::cout<<"How many circles in vector?"<<std::endl;
    // std::cin>>num_circles;
    // for (int i=1; i<=num_circles; i++){
    //     std::cout<<"Enter radius for circle "<<i<<": "<<std::endl;
    //     std::cin>>circle_radius;
    //     Circle circle(circle_radius);
    //     circleVec2.push_back(circle);
    // }
    // std::cout<<"Circle Vector created."<<std::endl;
    // std::cout<< "Total Area of circles in vector: "<<returnAreaOfCirclesVector(circleVec2)<<std::endl;
    
    // Program that creates a vector of circles with random radii (the radii can be between 1.0 and 10.0) and uses the function to compute the combined area.
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(1.0,10.0);

    int num_circles;
    std::vector<Circle> circleVec3;
    std::cout<<"How many circles in vector?"<<std::endl;
    std::cin>>num_circles;
    for (int i=1; i<=num_circles; i++){
        double randomRadii = distribution(generator);
        Circle circle(randomRadii);
        circleVec3.push_back(circle);
    }
    std::cout<<"Circle Vector created."<<std::endl;
    std::cout<< "Total Area of circles in vector: "<<returnAreaOfCirclesVector(circleVec3)<<std::endl;
}

