#include <iostream>
#include <vector>
#include <sstream>

int main(int argc, char** argv){
    int data_size = 10;

    std::vector<int> data; // vector of integers called data

    for (int i=0; i<data_size; i++){
        data.push_back(i); //populate vector using inbuilt push_back() function
    }

    for (int i=0; i<data.size(); i++){
        std::cout<<data.at(i)<<", "<<std::endl;;
    }
}