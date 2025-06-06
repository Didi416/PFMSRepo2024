#include <iostream>

// TASK
// color may be red (value 0), yellow (value 1), green (value 20), or blue (value 21)
enum color
{
    red,
    yellow,
    green = 20,
    blue = 21,
};

// altitude may be altitude::high or altitude::low
enum altitude: char
{
     high = 'h',
     low = 'l', // C++11 allows the extra comma
};

// TASK
// enumeration types can have overloaded operators
// add the missing types and a default case
std::ostream& operator<<(std::ostream& os, color c)
{
    switch(c)
    {
        case red   : os << "red";    break;
        case yellow: os << "yellow"; break;
        case green : os << "green";  break;
        case blue  : os << "blue";   break;
        default: os << "RAINBOW";
    }
    return os;
}

int main()
{
    color col = yellow;
    altitude a;
    a = altitude::low;

    std::cout << "col = " << col << std::endl;

    std::cout << "a = ";
    std::cout << static_cast<char>(a);
    std::cout << '\n';
}
