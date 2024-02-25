#include "sample.h"
#include <iostream>

int main () {


//  Create an object `sample` of `Sample` class
    Sample sample(2.0);
//  Display to standard output the value of parameter `value_` from the `sample` object.
    std::cout << "Value of parameter value:_" << sample.readValue() << std::endl;
    return 0;
}
 