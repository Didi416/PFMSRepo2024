#include <iostream>
#include "radar.h"

int main (void){

  // instantiate radar object
  Radar radar1;
  Radar radar2;
  Radar radar3;

  while (true){
    radar1.start();
    radar2.start();
    radar3.start();


    // for(auto elem : data){
    //   std::cout << elem << " ";
    // }
    // std::cout << std::endl;
  }

  return 0;
}

