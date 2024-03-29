#include "databuffer.h"
#include <random>
#include <chrono>
#include <thread>
#include <iostream>

DataBuffer::DataBuffer(double minVal, double maxVal,unsigned int trimSize):
    minVal_(minVal),maxVal_(maxVal),trimSize_(trimSize)
{
    //! @todo 1
    //! Start all the threads
    //! Syntax is
    //! thread_name = new std::thread(&ClassName::functionName,this)
    running_=true;
}

DataBuffer::~DataBuffer(){
}

void DataBuffer::removeValues(){
    while (running_) {
        // We use unique lock rather than locking and unlocking the mutex directly
        // http://www.cplusplus.com/reference/mutex/unique_lock/
        std::unique_lock<std::mutex> lck (mtx_);

        //! @todo Implement removing values in vector, less than `minVal`and greater than `maxVal`

        lck.unlock();
        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
}

void DataBuffer::addValues(vector<double> values){
  std::unique_lock<std::mutex> lck (mtx_);
  val_.insert(val_.end(), values.begin(),values.end());
  bubbleSort(val_);
}

void DataBuffer::trimLength(){

    while (running_) {
        std::unique_lock<std::mutex> lck (mtx_);
        //! @todo 
        //! How to trim length to `trimSize`

        lck.unlock();

        // This short delay prevents this thread from hard-looping and consuming too much cpu time
        // Using a condition_variable to make the thread wait provides a better solution to this problem
        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
  
}

vector<double> DataBuffer::getValues(void){
    std::unique_lock<std::mutex> lck (mtx_);
    return val_;
}


void DataBuffer::bubbleSort(vector<double>& vec){

    unsigned long n = vec.size();
    bool swapped = false;
    while (!swapped) {
        swapped = true;
        for (unsigned int i = 1; i<n;i++){
            if (vec.at(i-1) > vec.at(i)){
                std::swap(vec.at(i-1), vec.at(i));
                swapped = false;
            }
        }
    }

}