#include "ClearCore.h"

// IO-0 through A-12 are all available as digital inputs.
#define uncover_jog_button ConnectorDI6
#define cover_jog_button ConnectorDI7
#define auto_button ConnectorDI8
#define red_light ConnectorIO0
#define green_light ConnectorIO1
#define cover_rotary_switch_1 ConnectorIO2
#define cover_rotary_switch_2 ConnectorIO3

// The current state of the input pin
int16_t state;
#define motor_driver ConnectorM0
#define motor_cover_tube ConnectorM2
#define baudRate 9600
#define INPUT_A_FILTER 20 //Check this

bool lastInput1 = false;
bool lastInput2 = false;
bool estopActive = false;

unsigned long loop_start_time = 0;
const unsigned long loop_timeout = 90000;       //90 seconds
bool timed_out = false;                         //Variable to see if the auto cycle has timed out

int32_t uncoverPulses;
int32_t coverPulses;

double movement_length = 40; //ft
const double diam_cover = 6.923/12;
double ang_velocity_cover = 20*32;

// Define the velocity and acceleration limits to be used for each move for the driving motor
int velocityLimitRPM = 10;
int velocityLimit = velocityLimitRPM * 128000 / 60; // pulses per sec
int accelerationLimit = 6000; // pulses per sec^2
int velocityMCRPM = 9;
int velocityMC = velocityMCRPM * 32;

// Defines the limit of the torque command, as a percent of the motor's peak
//Ttorque Rating (must match the value used in MSP)....only for torque follower mode
double maxTorque = 100;

// For Velocity Follower Mode
double maxSpeedMC = 4000;
double inputVelocity = 100;

// Creates the Cycle count
int cycles = 0;

// Declares our user-defined helper functions on the bottom.
bool MoveDistance(int distance, MotorDriver& motor);
bool CommandVelocity(long commandedVelocity);
bool driver_stopped = true;
void setup() {
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1,
                          Connector::CPM_MODE_STEP_AND_DIR);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_M2M3,
                          Connector::CPM_MODE_A_DIRECT_B_PWM);
  // Set the motor's HLFB mode to bipolar PWM
  motor_driver.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motor_driver.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  // Sets the maximum velocity for each move
  motor_driver.VelMax(velocityLimit);
  // Set the maximum acceleration for each move
  motor_driver.AccelMax(accelerationLimit);
  // Sets up serial communication and waits up to 5 seconds for a port to open
  Serial.begin(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
      continue;
  }
 
  // Enables the motors
  motor_driver.EnableRequest(true);
  motor_cover_tube.EnableRequest(true);
 
//Define Pins
  uncover_jog_button.Mode(Connector::INPUT_DIGITAL);        
  cover_jog_button.Mode(Connector::INPUT_DIGITAL);
  auto_button.Mode(Connector::INPUT_DIGITAL);
  red_light.Mode(Connector::OUTPUT_DIGITAL);
  green_light.Mode(Connector::OUTPUT_DIGITAL);
  cover_rotary_switch_1.Mode(Connector::INPUT_DIGITAL);
  cover_rotary_switch_2.Mode(Connector::INPUT_DIGITAL);
  red_light.State(true);
  Serial.println("MAKE SURE THE POOL IS COVERED!!!!!");
}

void loop() {
 
   bool input1 = uncover_jog_button.State();
   bool input2 = cover_jog_button.State();
   bool uncoverPressed = input1 && !lastInput1;
   bool coverPressed = input2 && !lastInput2;
    lastInput1 = input1;
    lastInput2 = input2;
 
//||||||||||||||||||||||||||||||||||||||||||||||||||\\
// ***Start the auto testing sequence***
// ***ALWAYS START WHEN THE POOL IS COVERED!!!!!***
//||||||||||||||||||||||||||||||||||||||||||||||||||\\
 
  if (auto_button.State() == HIGH && timed_out == false) {
    
      //Serial.println("Auto on");                              //DEBUG
      motor_driver.EnableRequest(true);                         // Enable All Motors and Clear Alerts
      motor_cover_tube.EnableRequest(true);
      delay(1500);
      motor_driver.ClearAlerts();
      motor_cover_tube.ClearAlerts();
   
      //
      // ---------- UNCOVER ----------
      //
   
      green_light.State(true);
      velocityLimit = ang_velocity_cover * 128000 / 60 / 32;    // Set Speed of Driver Motor
      motor_driver.VelMax(velocityLimit);
      motor_driver.PositionRefSet(0);
   
      MoveDistance(10000000, motor_driver);                     // Start Driver Motor
      delay(6000);
      CommandVelocity(0, motor_cover_tube);                     // Start Cover Tube Motor
   
      //Serial.println("Uncovering the Pool");                  //DEBUG
      loop_start_time = millis();
   
      while (cover_rotary_switch_1.State() == HIGH && timed_out == false) {
       
        if (auto_button.State() == LOW){
          Serial.println("E-STOP");
          motor_driver.MoveStopDecel(10000);
          motor_cover_tube.MoveStopDecel(10000);
          CommandVelocity(0, motor_cover_tube);
          motor_driver.EnableRequest(false);
          motor_cover_tube.EnableRequest(false);
          break;
        }
       
        if (millis() - loop_start_time >= loop_timeout){
          Serial.println("TIMEOUT");
          motor_driver.MoveStopDecel(10000);
          motor_cover_tube.MoveStopDecel(10000);
          CommandVelocity(0, motor_cover_tube);
          motor_driver.EnableRequest(false);
          motor_cover_tube.EnableRequest(false);
          timed_out = true;
          break;
        }
       
        if (cover_rotary_switch_1.State() == LOW){
          motor_driver.MoveStopDecel(10000);
          motor_driver.EnableRequest(false);
          motor_cover_tube.EnableRequest(false);
        }
      }
   
      //Serial.println("The Pool is Uncovered");                //DEBUG
      green_light.State(false);
   
      // Stop motors (Take out once it has been verified)
      motor_driver.MoveStopDecel(10000);
      motor_cover_tube.MoveStopDecel(10000);
      CommandVelocity(0, motor_cover_tube);
      motor_driver.EnableRequest(false);
      motor_cover_tube.EnableRequest(false);
      delay(30000);                                             // MOTOR COOLDOWN
   
      //
      // ---------- COVER ----------
      //
   
      motor_driver.EnableRequest(true);                         // Enable All Motors and Clear Alerts
      motor_cover_tube.EnableRequest(true);
      delay(1500);
      motor_driver.ClearAlerts();
      motor_cover_tube.ClearAlerts();
      green_light.State(true);
      motor_driver.PositionRefSet(0);
      MoveDistance(-10000000, motor_driver);                    // Start Driver Motor
      delay(6000);
   
      CommandVelocity(-ang_velocity_cover, motor_cover_tube);   // Start Cover Motor
 
      //Serial.println("Covering the Pool");                    //DEBUG
      loop_start_time = millis();
   
      while (cover_rotary_switch_2.State() == HIGH && && timed_out == false) {
       
        if (auto_button.State() == LOW){
          Serial.println("E-STOP");
          motor_driver.MoveStopDecel(10000);
          motor_cover_tube.MoveStopDecel(10000);
          CommandVelocity(0, motor_cover_tube);
          motor_driver.EnableRequest(false);
          motor_cover_tube.EnableRequest(false);
          break;
        }
       
        if (millis() - loop_start_time >= loop_timeout){
          Serial.println("TIMEOUT");
          motor_driver.MoveStopDecel(10000);
          motor_cover_tube.MoveStopDecel(10000);
          CommandVelocity(0, motor_cover_tube);
          motor_driver.EnableRequest(false);
          motor_cover_tube.EnableRequest(false);
          timed_out = true;
          break;
        }
       
        if (cover_rotary_switch_2.State() == LOW){
          motor_cover_tube.MoveStopDecel(10000);
          motor_driver.MoveStopDecel(10000);
          CommandVelocity(0, motor_cover_tube);
          motor_cover_tube.EnableRequest(false);
          motor_driver.EnableRequest(false);
        }
      }
   
      //Serial.println("The Pool is Covered");                     //DEBUG
      // Stop motors (Take out once verified)
      motor_driver.MoveStopDecel(10000);
      motor_cover_tube.MoveStopDecel(10000);
      CommandVelocity(0, motor_cover_tube);
      motor_driver.EnableRequest(false);
      motor_cover_tube.EnableRequest(false);
 
      green_light.State(false);
      cycles += 1;
      Serial.print("Number of Cycles Completed: ");
      Serial.println(cycles);
   
      //Long Motor Cooldown
//      if (cycles == 33 || cycles == 67 || cycles == 100 || cycles == 133 || cycles == 167 || cycles == 200){
// 
//        //Serial.println("Finished 33 cycles; Resting for 1 minutes");       //DEBUG
//        delay(60000);
//      }
      delay(30000);                       // MOTOR COOLDOWN
  }

  //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
  // ***Jog Uncover the pool....cover is rolled up by driving motor***
  //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
   
  else if (uncoverPressed) {
   
    if (motor_driver.State() == MotorDriver::MOTOR_DISABLED || motor_cover_tube.State() == MotorDriver::MOTOR_DISABLED) {
      motor_driver.EnableRequest(true);
      motor_cover_tube.EnableRequest(true);
      delay(500);
      motor_driver.ClearAlerts();
      motor_cover_tube.ClearAlerts();
    }
    Comma
     ndVelocity(0, motor_cover_tube);
    velocityLimit = ang_velocity_cover * 128000 / 60 / 32;
    motor_driver.VelMax(velocityLimit);
    motor_driver.PositionRefSet(0);
    MoveDistance(10000000, motor_driver);
    delay(6000);
    while (uncover_jog_button.State() == HIGH) {
      continue;                                               // Keeps Loop Going Until uncover_jog_button.State() == LOW
    }
   
    uncoverPulses = motor_driver.PositionRefCommanded();
    Serial.print("Uncover Pulse Count Below:");
    Serial.println(uncoverPulses);
    driver_stopped = false;
  }
   
  //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
  // ***Jog Cover the pool....Cover is pulled by the cover motor***
  //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
   
  else if (coverPressed) {
   
    if (motor_driver.State() == MotorDriver::MOTOR_DISABLED || motor_cover_tube.State() == MotorDriver::MOTOR_DISABLED) {
      motor_driver.EnableRequest(true);
      motor_cover_tube.EnableRequest(true);
      delay(500);
      motor_driver.ClearAlerts();
      motor_cover_tube.ClearAlerts();
    }
 
    motor_driver.PositionRefSet(0);
    velocityLimit = ang_velocity_cover * 128000 / 60 / 32;
    motor_driver.VelMax(velocityLimit);
    MoveDistance(-10000000, motor_driver);
    delay(6000);
    CommandVelocity(-ang_velocity_cover, motor_cover_tube);
    while (cover_jog_button.State() == HIGH) {
      continue;
    }
   
    coverPulses = motor_driver.PositionRefCommanded();
    Serial.print("Cover Pulse Count Below:");
    Serial.println(coverPulses);
    driver_stopped = false;
  }
   
  //|||||||||||||||||||||\\
  //***STOP THE MOTORS***
  //|||||||||||||||||||||\\
   
  else {
   
    if (driver_stopped == false) {
      motor_driver.MoveStopDecel(10000);
      motor_cover_tube.MoveStopDecel(10000);
      motor_driver.MoveStopAbrupt();
      CommandVelocity(0, motor_cover_tube);
      driver_stopped = true;
      motor_driver.EnableRequest(false);
      motor_cover_tube.EnableRequest(false);
      delay(500);
    }
  }
}

bool MoveDistance(int distance, MotorDriver& motor) {
    // Serial.print("Moving distance: ");               //DEBUG
    // Serial.println(distance);
    // Command the move of incremental distance
    motor.Move(distance);
    return true;
}

bool CommandTorque(int commandedTorque, MotorDriver& motor) {
    if (abs(commandedTorque) > abs(maxTorque)) {
        Serial.println("Move rejected, invalid torque requested");
        return false;
    }
 
    // Check if an alert is currently preventing motion
    if (motor.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor status: 'In Alert'. Move Canceled.");
        return false;
    }
 
    Serial.print("Commanding torque: ");
    Serial.println(commandedTorque);
    // Find the scaling factor of our torque range mapped to the PWM duty cycle
    // range (255 is the max duty cycle).
    double scaleFactor = 255 / maxTorque;
    // Scale the torque command to our duty cycle range.
    int dutyRequest = abs(commandedTorque) * scaleFactor;
    // Set input A to match the direction of torque.
    if (commandedTorque < 0) {
        motor.MotorInAState(true);
    }
     
    else {
        motor.MotorInAState(false);
    }
    // Ensures this delay is at least 2ms longer than the Input A filter
    // setting in MSP
    delay(2 + INPUT_A_FILTER);
    // Command the move
    motor.MotorInBDuty(dutyRequest);
    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Moving... Waiting for HLFB");
    // Allow some time for HLFB to transition
    delay(1);
    Serial.println("Move Done");
    return true;
}

bool CommandVelocity(long commandedVelocity, MotorDriver & motor) {  
    if (abs(commandedVelocity) >= abs(maxSpeedMC)) {
        Serial.println("Move rejected, requested velocity at or over the limit.");
        return false;
    }
    // Check if an alert is currently preventing motion
    if (motor.StatusReg().bit.AlertsPresent) {
        //Serial.println("Motor status: 'In Alert'. Move Canceled.");
        return false;
    }
    // Change ClearPath's Input A state to change direction.
    // Note: this section of code was included so this commandVelocity function
    // could be used to command negative (opposite direction) velocity. However the
    // analog signal used by this example only commands positive velocities.
    if (commandedVelocity >= 0) {
        motor.MotorInAState(false);
    }
    else {
        motor.MotorInAState(true);
    }
    // Delays to send the correct filtered direction.
    delay(2 + INPUT_A_FILTER);
    // Find the scaling factor of our velocity range mapped to the PWM duty
    // cycle range (255 is the max duty cycle).
    double scaleFactor = 255 / maxSpeedMC;
    // Scale the velocity command to our duty cycle range.
    int dutyRequest = abs(commandedVelocity) * scaleFactor;
    // Command the move.
    motor.MotorInBDuty(dutyRequest);
    return true;
}
