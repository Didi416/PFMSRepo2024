// Includes std::cout and friends so we can output to console
#include <iostream>
#include <random>
#include "aircon.h"

// Every executable needs a main function which returns an int
int main (int argc,char** argv) {
    
    double desired_temp;
    double current_temp;
    std::cout<<"What is your desired temperature between 1 and 50 degrees?"<<std::endl;
    std::cin>>desired_temp;
    std::cout<<"Desired temperature: "<<desired_temp<<std::endl;

    long seed = std::chrono::system_clock::now().time_since_epoch().count(); //random number generator as used in functions.cpp
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(1.0,50.0);

    current_temp = distribution(generator);
    Aircon aircon(desired_temp, current_temp);
    std::cout<<"Current State: "<<aircon.state<<std::endl;
    std::cout<<"Current Temperature: "<<aircon.returnTemp()<<std::endl;

    system_clock::time_point startTime = system_clock::now();
    double runTime = 3*60^1000; //3000ms (3mins)

    while ((aircon.elapsedMs(startTime)< runTime) || (aircon.state != aircon::State::IDLE)){
        aircon.run();
    }
    return 0;
}
