#include "container_ops.h"
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>


void populateContainer(std::deque<double>& container, unsigned int num_values, double element){
    // std::cout<<"Change number of elements to : "<<std::endl;
    for (int i=0; i<num_values; i++){
        container.push_front(element);
    }
}
