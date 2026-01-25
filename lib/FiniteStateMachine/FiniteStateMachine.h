#ifndef _FINITE_STATE_MACHINE_H
#define _FINITE_STATE_MACHINE_H

#include "Mathpk.h"
#include "Constants.h"
enum class FlightState{
  BOOT,
  IDLE,
  ATTITUDE_STAND, //Mode for testing on attitude stand
  HOLD, //Altitude Hold
  WAYPOINT, 
  LAND
};


// FOR NOW, FSM WILL BE COMBINATION OF FLIGHT CONTROLLER + GUIDANCE.
// IT'LL HANDLE INFORMATION FLOW OF DIFFERENT CLASSES AND ALSO PROVIDE REFERENCE STATES TO THE CONTROLLER.
// WILL NEED TO EVENTUALLY BREAK THIS UP SO FSM SOLELY IS USED TO SWITCH ITS INTERNAL STATES
class FiniteStateMachine {
  public:
    FiniteStateMachine(float frequency)
    : updatePeriodMS_ (CONSTANTS::seconds2milli/frequency), 
      state_ (FlightState::BOOT), 
      timeInState_(0.0f), 
      timeEnterState_(0),
      lastUpdate_(0),
      commandControl_(false), 
      commandMotor_(false),
      criticalError_(false){}; 

    //Eventually want this to have more stuff...'
    // Like whether filter has converged, references set properly
    void preflightCheckStatus(uint32_t now, bool SensorCheckout) {
      if (state_==FlightState::BOOT && SensorCheckout) {
        transition(now, FlightState::IDLE); //Updates timeEnterState.

      }
    };

    void update(uint32_t now, Vector3f position, Vector3f velocity, Quaternion quaternion, Vector3f rates) {
      if ( (static_cast<float>(now - lastUpdate_) >= this->updatePeriodMS_) && (!criticalError_) ) {
        lastUpdate_ = now; 
        timeInState_ = static_cast<float>(now - timeEnterState_) / CONSTANTS::seconds2milli;

        // Catch any error first...
        //Calculate the tilt angle
        // Tilt angle is calculated by assuming the vector [0,0,1]. If this vector rotated by the quaternion results in
        // [x,y,z], this can be used to compare to the orignal vector to find the angle. 
        // std::array<float,3> tilt_vector {2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1-2*(qx^2+qy^2)}  //THis would be the vector rotated. But we need to dot this with the z axis, which just comes out to the last part
        float qx = quaternion.x();
        float qy = quaternion.y();
        float tilt_angle = acosf(1-2*(qx*qx + qy*qy));
        float tilt_angle_max =  PI/2; //If we go over 90 degrees 
        if (tilt_angle > tilt_angle_max) {
          transition(now, FlightState::IDLE);
          criticalError_ = true;
          return;
        }



        switch (this->state_) {
          case (FlightState::BOOT) : {
            //DO nothing. Should exit out of BOOT before using this function
            break;
          }
          case (FlightState::IDLE) : {
            if (timeInState_ > 5.0f) { //Idle for 5 seconds
              transition(now, FlightState::ATTITUDE_STAND); 
              //transition(now, FlightState::HOLD); 
            };
            break;
          };

          case (FlightState::ATTITUDE_STAND) :{
            if (timeInState_ > 60.0f) { //Test for set time
              transition(now, FlightState::BOOT); 
              //transition(now, FlightState::HOLD); 
            };
            break;
          }


          case (FlightState::HOLD) : {
            // Transition to this state should allow for control / motors
            if (timeInState_ < 10.0f) { //10 seconds to climb up 1 meter
              // Calculate a ramp input to the height
              //Only need to change position, the other references are from SETUP
              //Eventually move these calcs into guidance
              float maxHeight = -1.0f;
              float startingHeight = 0.0f; 
              float rampHeight = (maxHeight - startingHeight) / 10.0f * timeInState_;
              refPosition_=std::array<float,3>{0.0f, 0.0f, rampHeight}; 
              Serial.println(refPosition_[2]);
            } else if(timeInState_<20.0f && timeInState_ >=10.0f) { // Hold altitude for 10 seconds
              //Set the hieght reference to 1 meter if for some reason, the calculation doesn't end up getting to 1 meter in the ramp
              float maxHeight = -1.0f;
              refPosition_=std::array<float,3>{0.0f, 0.0f, maxHeight}; 
              Serial.println(refPosition_[2]);
            } else { //Exit out of this state after 20 seconds. KEEP the reference 
              transition(now, FlightState::WAYPOINT);
            };

            break;
          };

          case (FlightState::WAYPOINT) : {
            if (timeInState_ > 5.0f) {
              transition(now,FlightState::LAND);
            }

            break;
          };

          case (FlightState::LAND) : {
            if (timeInState_ > 5.0f) {
              transition(now,FlightState::BOOT); //Boot is the do nothing state for now
            }
            break;
          };

        };

      };
    };

    inline bool getMotorFlag() {
      return this->commandMotor_;
    }
    inline bool getControlFlag() {
      return this->commandControl_;
    }
  private:
    FlightState state_;
    uint32_t timeEnterState_;
    float timeInState_;  //Seconds
    float updatePeriodMS_;
    uint32_t lastUpdate_;

    Vector3f refPosition_;
    Vector3f refVelocity_;
    Quaternion refQuaternion_;
    Vector3f refRates_;

    bool commandControl_;
    bool commandMotor_;
    bool criticalError_;

    void transition(uint32_t now, FlightState newState) {
      if (newState != state_) {
        onExit(state_);
        state_ = newState;
        onEnter(now, state_);
      }
    };

    void onExit(const FlightState oldState) {
        switch (oldState) {
          case (FlightState::BOOT) :{
            if (Serial) {
              Serial.println("Exiting BOOT State");
              Serial.print("Time Spent in State: ");
              Serial.println(timeInState_);
            };
            break;
          };

          case (FlightState::IDLE) : {
            if (Serial) {
              Serial.println("Exiting IDLE State");
              Serial.print("Time Spent in State: ");
              Serial.println(timeInState_);
            };
            break;
          };

          case (FlightState::ATTITUDE_STAND) : {
            if (Serial) {
              Serial.println("Exiting ATTITUDE_STAND State");
              Serial.print("Time Spent in State: ");
              Serial.println(timeInState_);
            };
            break;
          };

          case (FlightState::HOLD) : {
            if (Serial) {
              Serial.println("Exiting HOLD State");
              Serial.print("Time Spent in State: ");
              Serial.println(timeInState_);
            };
            break;
          };

          case (FlightState::WAYPOINT) : {
            if (Serial) {
              Serial.println("Exiting WAYPOINT State");
              Serial.print("Time Spent in State: ");
              Serial.println(timeInState_);
            };
            break;
          };

          case (FlightState::LAND) : {
            if (Serial) {
              Serial.println("Exiting LAND State");
              Serial.print("Time Spent in State: ");
              Serial.println(timeInState_);
            };
            break;
          };
        };
    };



    void onEnter(const uint32_t now, const FlightState newState) {
        switch (newState) {
          case (FlightState::BOOT) : { //Using this state as IDLE. For testing, IDLE is more of a delay
            if (Serial) {
              Serial.println("Entering BOOT State");
            };
            timeEnterState_ = now;
            timeInState_ = 0.0f;
            lastUpdate_ = now;
            stateAllowsControl(false);
            break;
          }

          case (FlightState::IDLE) : {
            if (Serial) {
              Serial.println("Entering IDLE State");
            };
            timeEnterState_ = now;
            timeInState_ = 0.0f;
            lastUpdate_ = now;

            stateAllowsControl(false);
            break;
          };

          case (FlightState::ATTITUDE_STAND) : {
            if (Serial) {
              Serial.println("Entering ATTITUDE_STAND State");
            };
            timeEnterState_ = now;
            timeInState_ = 0.0f;
            lastUpdate_ = now;
            stateAllowsControl(true);

            break;
          };

          case (FlightState::HOLD) : {
            if (Serial) {
              Serial.println("Entering HOLD State");
            };
            timeEnterState_ = now;
            timeInState_ = 0.0f;
            lastUpdate_ = now;
            stateAllowsControl(true);

            break;
          };

          case (FlightState::WAYPOINT) : {
            if (Serial) {
              Serial.println("Entering WAYPOINT State");
            };
            timeEnterState_ = now;
            timeInState_ = 0.0f;
            lastUpdate_ = now;
            stateAllowsControl(true);
            break;
          };

          case (FlightState::LAND) : {
            if (Serial) {
              Serial.println("Entering LAND State");
            };
            timeEnterState_ = now;
            timeInState_ = 0.0f;
            lastUpdate_ = now;
            stateAllowsControl(true);
            break;
          };
        };
    };

    //Used whenever entering new state
    void resetTiming(uint32_t now) {
      timeEnterState_ = now; //Self-explanatory, need to set this to now when we enter a new state
      lastUpdate_ = now; //Need to reset the last update to now. This should always be done, but catches the edge case where we enter from boot --> idle, 
      timeInState_ = 0.0f;
    }


    void stateAllowsControl(bool controlMotorBool) {
      if (controlMotorBool) {
        if (!commandControl_) {
          commandControl_ = true;
        }
        if (!commandMotor_) {
          commandMotor_ = true;
        }
      } else {
        if (commandControl_) {
          commandControl_ = false;
        }
        if (commandMotor_) {
          commandMotor_ = false;
        }
      }
    }

};

#endif