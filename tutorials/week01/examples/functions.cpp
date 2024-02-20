// Includes std::cout and friends so we can output to console
#include <iostream>
// Random number
#include <random>
// chrono for time (seed))
#include <chrono>
// The sensor
#include "sensor.h"

// Ex01. Returns a bool value if the double is greater than zero
// and the square value instead of initial passed value
bool squareOfCheckPositive(double &value) {
    bool is_positive = value > 0.0;
    value *= value;
    return is_positive;
}

// Ex02. Returns bool value if the double is greater than zero, the square value, the cube value and the passed value incremented by one
void randomize(Sensor& sensor) {
 long seed = std::chrono::system_clock::now().time_since_epoch().count(); //This is a seed for our random number generator, otherwise would always have the same "random" number when it starts
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(1.0,50.0);// unfirom distrbution betwen 1 to 50

    for(unsigned int i = 0; i < sensor.num_samples; i++) {
        sensor.data[i] = distribution(generator);
    }

}


// ADVANCED: How best protect the passed value?

// Every executable needs a main function which returns an int
int main () {

    double x = 2.0;


    // Ex01.
    double y = x;
    std::cout << x << " is ";
    bool result = squareOfCheckPositive(y);
    if (result) {
        std::cout << "positive ";
    } else {
        std::cout << "not positive ";
    }
    std::cout << "and its square is " << y << std::endl;

    // Ex02.
    //Create a sensor with Sensor type structure
    Sensor sensor;
    //Let's say we have 5 samples ...
    sensor.num_samples=5;
    randomize(sensor);

    return 0;
}




