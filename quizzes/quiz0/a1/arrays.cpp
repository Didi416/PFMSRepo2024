
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include "arrays.h"
#include <iostream>
#include <iterator>
#include <algorithm>

#include <cstring> //for memcpy example

// thread and chrono are for time and sleeping respectively - added for class
#include <chrono>
#include <thread>
using namespace std::chrono;//Let's us refer to system_clock without stating std::chrono

// function to populate array with random numbers
void populateWithRandomNumbers(double x[], unsigned int& array_size, unsigned int num_elements) {

    //we select a seed for the random generator, so it is truly random (never the same seed)
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a normal distribution with means zero and standard deviation 10
    std::normal_distribution<> distribution(0,10.0);
    // generate the required amount of random numbers
    for (unsigned int i=array_size; i<array_size+num_elements; i++) {
        x[i] = distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}


//1) TASK: Implement function `assignArrayToVector`that assigns elements of array `x` to a vector named `vec`
void assignArrayToVector(double x[] ,unsigned int arraySize ,std::vector<double> &myVec ){
    /*double n = sizeof(x)/sizeof(x[0]);
    int N = static_cast<int>(n);
    myVec.insert(myVec.begin(),x,x+N); */

    for (int i=0; i<arraySize; i++){
        myVec.push_back(x[i]);
    }

}

//2) TASK: Implement function `removeNumbersLargerThan` that accepts vector `myVec` and removes elements of `myVec` greater than a `limit`
void removeNumbersLargerThan(std::vector<double> &myVec, double limit){
    std::cout << "Limit: " << limit << std::endl;
    for (int i=0; i<myVec.size(); i++){
        auto elem = myVec.begin()+i;
        std::cout << i << ": " << *elem << std::endl;
        if (*elem > limit){
            std::cout << "Removing: " << *elem << std::endl;
            myVec.erase(myVec.begin()+i);
            --i;
        }
    }
    std::cout << myVec.size() << std::endl;
}


//3) TASK: Implement function `computeMeanAndStdDev` that computes the mean and standard deviation of `myVec` and returns the `Stats` structure with these values.
Stats computeMeanAndStdDev(std::vector<double> myVec){
    Stats stats;
    //Mean
    double mu = accumulate(myVec.begin(), myVec.end(), 0.0)/myVec.size();
    std::cout << "Mean: " << mu << std::endl;
    stats.mean = mu;
    
    //Standard Deviation
    double sumSquaredDiffs = 0;
    double sd;
    for (int i=0; i<myVec.size(); i++){
        auto elem = myVec.begin()+i;
        double diff = *elem - mu;
        sumSquaredDiffs += diff * diff;

        sd = sqrt(sumSquaredDiffs/myVec.size());
    }
    std::cout << "Standard Deviation: " << sd << std::endl;
    stats.std_dev = sd;

    return stats;
}

//4) TASK:  Implement function `returnVecWithNumbersSmallerThan` that returns a vector containing elements of `myVec` vector that are less than `limit`.
std::vector<double> returnVecWithNumbersSmallerThan(std::vector<double> myVec, double limit){
    double value;
    std::vector<double> smallNumVec;
    for (int i=0; i<myVec.size(); i++){
        auto elem = myVec.begin()+i;
        if (*elem < limit){
            value = *elem;
            smallNumVec.push_back(value);
        }
    }
    return smallNumVec;
    //BONUS - Can you use a lambda function instead of looping?

}

