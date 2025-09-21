#ifndef SENSORS_H
#define SENSORS_H

#include "Sensors.h"

class FSM {
public: 
    enum class State{
        Idle, //This is what the Drone starts off in
        Checkout, //This is where the drone runs self-tests, verifying sensors, starting filter, etc
        Error, //Failure during checkout. Or an unrecoverable fault is detected
        Armed, //This is when all checks have been done and its ready to fly
        Active, //This state has the drone actively executing guidance
        Safe, // Mode where an anomaly (small fault) is detected and all guidance is cancelled in order to maintain current height and identity quaternion
        Return, //Mode where the drone either returns to initial waypoint if no major anomaly
        Desend //Mode where drone slowly descends from current height 
    };

    //Types of errors / faults
    enum class Error {

    };
    //Constructor. States off the drone in Idle.
    FSM() : state(State::Idle) {};

    // Function to check the state triggers
    void update(Sensors& sensors) {
        switch (state) {
            case State::Idle:
                break;
            case State::Checkout:
                break;

            case State::Armed:
                break;
            case State::Active:
                break;
            case State::Safe:
                break;
            case State::Return:
                break;
            case State::Desend:
                break;
            case State::Error:
                break;
            
        }

    };

    State getState() {
        return state;
    }

private: 
    State state;
    Error error;


}; 
#endif //FSM