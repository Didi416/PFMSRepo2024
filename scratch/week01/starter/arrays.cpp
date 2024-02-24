// Includes std::cout and friends so we can output to console
#include <iostream>

// Create a macro (#define) to represent the array size
#define ARRAY_SIZE 10

// Every executable needs a main function which returns an int
int main () {

    // Ex02
    
    // Create an array x of doubles with 10 elements
    // Populate the elements of array on creating of array, each element [i] has value -1 (HINT: INITIALISER LIST)
    double x[ARRAY_SIZE] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    // Can you create a loop to populate elements of x (each x[i] =i), how to code end of loop?) – (HINT: USE MACRO ARRAY_SIZE)
    /*for ( int i=0 ; i<(ARRAY_SIZE) ; i++ ) {
        x[i] = i;
        std::cout << "Value: " << x[i] << std::endl;
    }*/
    // Can you use a pointer and loop to initialise elements of array
    double *ip = x;
    for ( int i=0 ; i<(ARRAY_SIZE) ; i++ ) {
        *ip = i;
        std::cout << "Value: " << x[i] << std::endl;
        ++ip;
    }
    // Main function should return an integer
    return 0;
}
