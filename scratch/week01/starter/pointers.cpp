#include <iostream>

// Pointers are essential to C++ and are used to store the memory address of another variable.
// They are declared by using the asterisk (*) before the variable name.
// The data type of the pointer must be the same as the data type of the variable you're storing the address of.
// When passed as a parameter to a function, the function can modify the value of the variable by using the pointer.

int main () {

    // Ex01
//    * Assign to variable x (of type double) the value 41012
    double x = 41012;
    std::cout << "x = " << x << std::endl;

//    * Use a pointer ip to point to x (why type is ip?)
    double *ip = &x;
//    * Print the value of what ip is pointing to
//    * Print the value of ip
    printf("Value for x is %If\n", *ip);
    printf("Value for *ip is %If\n", x);
//    * Make variable y a reference to x (what type is y?)
    double &y = x;
//    * Print the value of what y is referencing to
    printf("Value of y is %If\n", y);
//    * Create a double z of value 1
    double z = 1;
//    * Use pointer ip to point to z
    double *ipz = &z;
//    * Assign z the value 100
    z = 100;
//    * Print the value of what ip is pointing to
//    * Print the value of x
//    * Print the value of y
//    * Print the value of z
    printf("Value for *ipz is %If\n", *ipz);
    printf("Value for z is %If\n", x);
    printf("Value for y is %If\n", y);
    printf("Value for z is %If\n", z);
    return 0;
}
