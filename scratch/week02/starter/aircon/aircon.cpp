#include <iostream>
#include <random>
#include "aircon.h"
#include <thread>
#include <cmath>
#include <random>
#include <sstream>

Aircon::Aircon(double desired_temp, double current_temp){
    _desired_temp = desired_temp;
    _current_temp = current_temp;
    state = aircon::State::IDLE;
}

void Aircon::heating(){
    _current_temp += 0.5;
}

void Aircon::cooling(){
    _current_temp -= 0.5;
}

double Aircon::returnTemp(){
    // Temps temps;
    // temps.heating_active = _heating_active;
    double current_temp = _current_temp;
    // temps.desired_temp = _desired_temp;
    return current_temp;
}

double Aircon::elapsedMs(system_clock::time_point start_time){
    return duration_cast<milliseconds>(system_clock::now() - start_time).count();
}

void Aircon::sleepMs(int ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

bool Aircon::tempReached(){
    return std::fabs(_current_temp - _desired_temp) < 0.5;
}

void Aircon::run(){
    switch(state){
        case aircon::State::IDLE : {
            if((_current_temp-_desired_temp)>0.5){
                state=aircon::State::COOLING;
            }
            else if((_desired_temp-_current_temp)>0.5){
                state=aircon::State::HEATING;
            }
            break;
        }
        case aircon::State::HEATING : {
            if (tempReached()){
                state=aircon::State::IDLE;
                break;
            }
            if(_heating_active){
                heating();
            }
            else{
                _heating_active=true;
            }
            break;
        }
        case aircon::State::COOLING : {
            if (tempReached()){
                state=aircon::State::IDLE;
                break;
            }
            cooling();
            break;
        }
    }
    std::cout<<"Current Temperature: "<<returnTemp()<<std::endl;
    sleepMs(1000);//Sleep for 5s (in ms)
}


