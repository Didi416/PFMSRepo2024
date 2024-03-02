#ifndef AIRCON_H
#define AIRCON_H

#include <chrono>
#include <string>

using namespace std::chrono;//Let's us refer to system_clock without stating std::chrono

// Aircon unit has a few states
namespace aircon{
    enum State
    {
        IDLE,       // Neither heating nor cooling
        COOLING,    // Cooling
        HEATING     // Heating
    };
}

class Aircon
{
public:
    /**
     * @brief Creates aircon, defaults to running it for 3minutes, 
     * Requires executing run function to peform task, becuase we do not know threading yet :)
     */
    aircon::State state;
    Aircon(double desired_temp, double current_temp);
    void heating();
    void cooling();
    double returnTemp();
    double elapsedMs(system_clock::time_point start_time);
    void sleepMs(int ms);
    bool tempReached();
    void run();

    bool _heating_active;
    double _desired_temp;
    double _current_temp;
};

#endif // AIRCON_H
