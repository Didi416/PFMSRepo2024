// Includes std::cout and friends so we can output to console
#include <iostream>
#include "sensor.h"
// We will introduces classes later
// - To protect data and ensure integirty (access specifiers)
// - To embed data and functionality together
// - To ensure the container is safe to use on ititialisation (constructor)

void print_struct(Sensor sensor){
  //! Loop to print elements of data in struct (let's acess each element location
    for (unsigned int i=0;i<sensor.num_samples;i++){
        std::cout << sensor.data[i] << " ";
    }
    std::cout << std::endl;
}


// Every executable needs a main function which returns an int
int main (int argc,char** argv) {

    //We create sensor variable of Sensor type
    Sensor sensor;

    //Ex02 
    //Initialise the structure with 5 samples (the values can be any values you choose)
    sensor.num_samples = 5;
    for (int i=0; i<sensor.num_samples; i++){
        sensor.data[i] = i;
        std::cout << "Data Point: " << sensor.data[i] << std::endl;
    }
    //Function called to print variable
    print_struct(sensor);

    //Now create sensor2 of Sensor type using list initalisation (aggregate initialisation)
    Sensor sensor2 = {5, {2,7,8,5,3}};
    //https://en.cppreference.com/w/cpp/language/list_initialization
    //https://en.cppreference.com/w/cpp/language/aggregate_initialization

    //Function called to print variable
    print_struct(sensor2);

    return 0;
}




