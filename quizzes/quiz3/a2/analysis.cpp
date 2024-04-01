#include "analysis.h"

#include <iostream> // Only here for showing the code is working
#include <thread>

Analysis::Analysis(std::shared_ptr<Radar> radarPtr):
    radarPtr_(radarPtr)
{

}

//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
void Analysis::computeScanningSpeed(unsigned int samples, double& scanningSpeed){
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  for (unsigned long i=0; i<samples; i++){
    radarPtr_->getData();
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time = end - start;

  scanningSpeed = samples/time.count()*(1.0/100.0);
  return;
}
