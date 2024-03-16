#include "analysis.h"
#include <algorithm>
#include <chrono>

using std::vector;
using std::pair;
using namespace std::chrono;

Analysis::Analysis(std::vector<CarInterface*> cars) :
    cars_(cars),raceDisplay_(nullptr)
{

}

Analysis::Analysis(std::vector<CarInterface*> cars,std::shared_ptr<DisplayRace> raceDisplay) :
    cars_(cars),raceDisplay_(raceDisplay)
{

}


//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::dragRace(double distance){

    std::vector<unsigned int> order(cars_.size(),0);
    std::vector<CarInterface*>::iterator iter;
    std::vector<double> initialOdo;
    int idx;
    bool finished = false;
    std::vector<int> x(3, -1);
    unsigned int size = 0;

    for (iter = cars_.begin(); iter != cars_.end(); iter++){
        if (size < cars_.size()){
                initialOdo.push_back((*iter)->getOdometry());
                size++;
            }
    }
    while(!finished){
        idx = -1;
        for (iter = cars_.begin(); iter != cars_.end(); iter++){
            idx++;
            (*iter)->accelerate();
            
            if (((*iter)->getOdometry()-initialOdo[idx])>=distance && !(std::count(order.begin(),order.end(),(*iter)->getID()))){
                for (int i=0; i<order.size(); i++){
                    if(order.at(i) == 0){
                        order[i] = (*iter)->getID();
                        std::cout<<order[i]<<std::endl;
                        break;
                    }
                }
                
                if (!(std::count(order.begin(),order.end(),0))){
                    finished = true;
                } 
            }
        }
    }
    return order;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars(){
    std::vector<int> x(3, -1);
    std::vector<CarInterface*>::iterator iter;
    bool finished = false;

    while(!finished){
        for (iter = cars_.begin(); iter != cars_.end(); iter++){
            if (!(std::count(x.begin(),x.end(),(*iter)->getID()))){
                (*iter)->decelerate();
                if ((*iter)->getCurrentSpeed() <= 0){
                    for (int i=0; i<x.size(); i++){
                        if(x.at(i) == -1){
                            x[i] = (*iter)->getID();
                            break;
                        }
                    }
                }
            }

            if (x.size() == cars_.size() && !(std::count(x.begin(),x.end(), -1))){
                finished = true;
                break;
            }
        }
    }
    return;
}

//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::zeroTopZeroRace(){

    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros
    std::vector<CarInterface*>::iterator iter;
    int idx;
    bool finished = false;
    std::vector<int> decelerate(cars_.size(), -1);
    unsigned int size = 0;
    
    while(!finished){
        idx = -1;
        for (iter = cars_.begin(); iter != cars_.end(); iter++){
            idx++;

            if(!(std::count(order.begin(),order.end(),(*iter)->getID()))){
                if((*iter)->getCurrentSpeed() < (*iter)->getTopSpeed() && !(std::count(decelerate.begin(),decelerate.end(),(*iter)->getID()))){
                    (*iter)->accelerate();
                    std::cout<<(*iter)->getID()<<" is Accelerating at "<<(*iter)->getCurrentSpeed()<<std::endl;
                }

                else if ((*iter)->getCurrentSpeed() >= (*iter)->getTopSpeed()){
                    if(!std::count(decelerate.begin(),decelerate.end(),(*iter)->getID())){
                        for (int i=0; i<decelerate.size(); i++){
                            if(decelerate.at(i) == -1){
                                decelerate[i] = (*iter)->getID();
                                std::cout<<decelerate[i]<<std::endl;
                                break;
                            }
                        }
                    }
                }

                if ((std::count(decelerate.begin(),decelerate.end(),(*iter)->getID()))){
                    (*iter)->decelerate();
                    std::cout<<(*iter)->getID()<<" is Decelerating"<<std::endl;
                    if((*iter)->getCurrentSpeed() <= 0){
                        for (int i=0; i<order.size(); i++){
                            if(order.at(i) == 0){
                                order[i] = (*iter)->getID();
                                std::cout<<order[i]<<std::endl;
                                break;
                            }
                        }
                    }
                }
            }
            
            if (!(std::count(order.begin(),order.end(),0))){
                    finished = true;
            }
        }
    }
    return order;
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::sortByOdometry(){

    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros
    std::vector<double> odoReadings;
    std::vector<std::pair<int, double>> pairs;
    std::vector<CarInterface*>::iterator iter;

    for (iter = cars_.begin(); iter != cars_.end(); iter++){
        odoReadings.push_back((*iter)->getOdometry());
        std::cout<<(*iter)->getID()<<" odometry is "<<(*iter)->getOdometry()<<std::endl;
        for (int i=0; i<order.size(); i++){
            if(order.at(i) == 0){
                order[i] = (*iter)->getID();
                break;
            }
        }
    }

    for (int i=0; i<order.size(); i++){
        pairs.push_back(std::make_pair(order[i], odoReadings[i]));
    }

    for (iter = cars_.begin(); iter != cars_.end(); iter++){
        std::sort(pairs.begin(), pairs.end(), [](std::pair<int, double> &a, std::pair<int, double> &b){return a.second > b.second;});
    }

    for (int i=0; i<pairs.size(); i++){
        order[i] = pairs[i].first;
        std::cout<<order[i]<<std::endl;
    }

    return order;
}

// Demo code
void Analysis::demoRace(){


    //This is an example of how to draw 3 cars moving
    // accelerate 300 times
    unsigned int count=0;

    while(count < 300){
        for(auto car : cars_){
          car->accelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

    // decelerate 600 times
    count =0;
    while(count < 600){
        for(auto car : cars_){
          car->decelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

}
