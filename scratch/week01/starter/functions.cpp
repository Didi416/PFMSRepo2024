// Includes std::cout and friends so we can output to console
#include <iostream>
#include <random> //random
#include <chrono> //time
#include "sensor.h"

// Functions are the building blocks of any programming language. They allow us to encapsulate a block of code
// and execute it whenever we want. This is useful for code reuse and readability.
//
// Functions are defined with the following syntax:
// <return type> <function name> (<parameters>) {
//     <function body>
// }


// Ex01. 
//* Create a function named 'square` that accepts a double value as a parameter and
//* Returns a bool value if the double is greater than zero 
//* The parameter passed needs to be squared, so it get's returned as a square value to the caller

bool square(double &value){
    bool isPositive = value > 0;
    value *= value;
    return isPositive;
}

//* Create an function named `randomize` that accepts a structure `Sensor` and populates data with random values (mean 5.0 standard deviation 3.0)
void randomize(Sensor &sensor){
    long seed = std::chrono::system_clock::now().time_since_epoch().count(); 
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(1.0,50.0);// unfirom distrbution betwen 1 to 50

    for(unsigned int i = 0; i < sensor.num_samples; i++) {
        sensor.data[i] = distribution(generator);
    }
}
// Every executable needs a main function which returns an int
int main () {

	//Call the functions, obtain values and print to screen (using std::cout) 
    double x = 4.0;
    double y = x;
    bool result = square(y);
    
    if (result == 1){
        std::cout<<"Result is POSITIVE";
    }
    else{
        std::cout<<"Result is NOT POSITIVE";
    }

    Sensor sensor;
    sensor.num_samples = 4;
    randomize(sensor);
    return 0;
}