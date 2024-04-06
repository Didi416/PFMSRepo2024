#include "analysis.h"
#include <iostream>
#include <thread>
#include <mutex>

using std::cout;
using std::endl;
using std::mutex;

Analysis::Analysis(pfms::Data* data) :
    data_(data)
{
    // Create the threads
    running_=true;
    thread_ = new std::thread(&Analysis::processSamples,this);

}

Analysis::~Analysis()
{
    running_=false;
    // Destruct the threads
    thread_->join();
    delete thread_;
}

bool Analysis::isRunning(){
    return running_;

}

// This function consumes the samples
void Analysis::processSamples() {
    while (true) {

        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        std::unique_lock<mutex> lk(data_->mtx);
        data_->cv.wait(lk, [this]{return !this->data_->data.empty();});

        int sample = data_->data.back();
        data_->data.pop_back();
        cout << "sample:" << sample << " fibonacci:" << fibonacci(sample) << endl;
        lk.unlock(); // We release mutex here as we don't 
    }
}

//recursive function for fibonacci
int Analysis::fibonacci(int n)
{
    //if n is zero or one return the number
    if(n<=1)
    {
        return n;
    }
    //recursive call to n-1 and n-2 
    return fibonacci(n-1)+fibonacci(n-2);
}