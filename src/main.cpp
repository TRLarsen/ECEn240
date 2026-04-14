/********************************************************************
  ECEN 240/301 Lab Code
  Light-Following Robot

  The approach of this code is to use an architectured that employs
  three different processes:
    Perception
    Planning
    Action

  By separating these processes, this allows one to focus on the
  individual elements needed to do these tasks that are general
  to most robotics.


  Version History
  1.1.3       11 January 2023   Creation by Dr. Mazzeo and TAs from 2022 version

 ********************************************************************/

/* These initial includes allow you to use necessary libraries for
your sensors and servos. */
#include <Arduino.h>
#include <CapacitiveSensor.h>
#include <Servo.h>  // loads the Servo library
#include <NewPing.h>

//
// Compiler defines: the compiler replaces each name with its assignment
// (These make your code so much more readable.)
//

/***********************************************************/
// Hardware pin definitions
// Replace the pin numbers with those you connect to your robot

// Button pins. These will be replaced with the photodiode variables in lab 5
#define BOTTOM_OUTSIDE_PHOTODIODE  A2     // bottom outside
#define UPPER_INSIDE_PHOTODIODE  A3     // upper inside
// #define BUTTON_3  A4     // Middle Button - Collision
#define UPPER_OUTSIDE_PHOTODIODE  A5     // upper outside
#define BOTTOM_INSIDE_PHOTODIODE  A6     // bottom inside

// LED pins (note that digital pins do not need "D" in front of them)
// #define LED_1   6       // Far Left LED - Servo Up
// #define LED_2   5       // Left Middle LED  - Left Motor
// #define LED_3   4       // Middle LED - Collision
// #define LED_4   3       // Right Middle LED - Right Motor
// #define LED_5   2       // Far Right LED - Servo Down
#define RED_PIN 9
#define GREEN_PIN 10
#define BLUE_PIN 11

// Motor enable pins - Lab 3
#define H_BRIDGE_ENA  5
#define LEFT_MOTOR    5       // Left Motor
#define RIGHT_MOTOR   3       // Right Motor
#define H_BRIDGE_ENB  3

// Photodiode pins - Lab 5
// These will replace buttons 1, 2, 4, 5

// Capacitive sensor pins - Lab 4
#define CAP_SENSOR_SEND    4
#define CAP_SENSOR_RECEIVE 2

// Ultrasonic sensor pin - Lab 6
// This will replace button 3 and LED 3 will no longer be needed
#define TRIGGER_PIN 7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 8  // Arduino pin tied to echo pin on the ultrasonic sensor.

// Servo pin - Lab 6
#define SERVO_PIN 6

/***********************************************************/
// Configuration parameter definitions
// Replace the parameters with those that are appropriate for your robot

// Voltage at which a button is considered to be pressed
#define BUTTON_THRESHOLD 2.5

// Voltage at which a photodiode voltage is considered to be present - Lab 5
#define PHOTODIODE_LIGHT_THRESHOLD 3

// Number of samples that the capacitor sensor will use in a measurement - Lab 4
#define CAP_SENSOR_SAMPLES       40
#define CAP_SENSOR_TAU_THRESHOLD 35

// Parameters for servo control as well as instantiation - Lab 6
// Note that the a lower angle is towards the back of the robot
#define SERVO_START_ANGLE 90
#define SERVO_UP_LIMIT 100
#define SERVO_DOWN_LIMIT 60
static Servo myServo;

// Parameters for ultrasonic sensor and instantiation - Lab 6
// Maximum distance we want to ping for (in centimeters). 
// Maximum sensor distance is rated at 400-500cm, so we choose 200.
#define MAX_DISTANCE 200 

// NewPing setup of pins
static NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

// Parameter to define when the ultrasonic sensor detects a collision - Lab 6
#define COLLISION_DISTANCE 20


/***********************************************************/
// Defintions that allow one to set states
// Sensor state definitions
#define DETECTION_NO    0
#define DETECTION_YES   1

// Motor speed definitions - Lab 4
#define SPEED_STOP      0
#define SPEED_LOW       (int) (255 * 0.45)
#define SPEED_MED       (int) (255 * 0.75)
#define SPEED_HIGH      (int) (255 * 1)


// Collision definitions
#define COLLISION_ON   0
#define COLLISION_OFF  1

// Driving direction definitions
#define DRIVE_STOP      0
#define DRIVE_LEFT      1
#define DRIVE_RIGHT     2
#define DRIVE_STRAIGHT  3

// Servo movement definitions
#define SERVO_MOVE_STOP 0
#define SERVO_MOVE_UP   1
#define SERVO_MOVE_DOWN 2


/***********************************************************/
// Global variables that define PERCEPTION and initialization

// Collision (using Definitions)
int SensedCollision = DETECTION_NO;

// Photodiode inputs (using Definitions) - The button represent the photodiodes for lab 2
int SensedLightRight = DETECTION_NO;
int SensedLightLeft = DETECTION_NO;
int SensedLightUp = DETECTION_NO;
int SensedLightDown = DETECTION_NO;
int SensedLight = DETECTION_NO;

// Capacitive sensor input (using Definitions) - Lab 4
int SensedCapacitiveTouch = DETECTION_NO;


/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int ActionCollision = COLLISION_OFF;

// Main motors Action (using Definitions)
int ActionRobotDrive = DRIVE_STOP;
int ActionRobotSpeed = SPEED_STOP;

// Servo Action (using Definitions)
int ActionServoMove =  SERVO_MOVE_STOP;

// Struct defining what RGB LED PWM values to write
struct RGB {
  uint8_t redPwm = 0;
  uint8_t greenPwm = 0;
  uint8_t bluePwm = 0;
} rgbVals;

/********************************************************************
  SETUP function - this gets executed at power up, or after a reset
 ********************************************************************/
void setup() {
  //Set up serial connection at 9600 Baud
  Serial.begin(9600);
  
  //Set up output pins
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(CAP_SENSOR_SEND, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  
  //Set up input pins
  pinMode(BOTTOM_OUTSIDE_PHOTODIODE, INPUT);
  pinMode(UPPER_INSIDE_PHOTODIODE, INPUT);
  // pinMode(BUTTON_3, INPUT);
  pinMode(UPPER_OUTSIDE_PHOTODIODE, INPUT);
  pinMode(BOTTOM_INSIDE_PHOTODIODE, INPUT);

  // Capacitive Sensor
  pinMode(CAP_SENSOR_RECEIVE, INPUT);
  pinMode(CAP_SENSOR_SEND, OUTPUT);

  // Ultrasonic Sensor
  pinMode(TRIGGER_PIN, OUTPUT); // pulse sent out through TRIGGER_PIN    
  pinMode(ECHO_PIN, INPUT); // return signal read through ECHO_PIN

  // // Battery sensor
  // pinMode(A1, INPUT);
  // pinMode(10, OUTPUT);
  // pinMode(11, OUTPUT);
  // pinMode(12, OUTPUT);

  //Set up servo - Lab 6
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_START_ANGLE);
}

/**********************************************************************************************************
  AUXILIARY functions that may be useful in performing diagnostics
 ********************************************************************/
// Function to turn LED on
void doTurnLedOn(int led_pin)
{
  digitalWrite(led_pin, HIGH);
}

// Function to turn LED off
void doTurnLedOff(int led_pin)
{
  digitalWrite(led_pin, LOW);
}

// Simple non-blocking timer. Requires user to keep track of start time.
unsigned long elapsedTime(unsigned long startTime){
  return (millis() - startTime);
}

////////////////////////////////////////////////////////////////////
// Function to read pin voltage
////////////////////////////////////////////////////////////////////
float getPinVoltage(int pin) {
  //This function can be used for many different tasks in the labs
  //Study this line of code to understand what is going on!!
  //What does analogRead(pin) do?
  //Why is (float) needed?
  //Why divide by 1024?
  //Why multiply by 5?
  return 5 * (float)analogRead(pin) / 1024;
}

////////////////////////////////////////////////////////////////////
// Function to determine if a button is pushed or not
////////////////////////////////////////////////////////////////////
bool isButtonPushed(int button_pin) {
  //This function can be used to determine if a said button is pushed.
  //Remember that when the voltage is 0, it's only close to zero.
  //Hint: Call the getPinVoltage function and if that value is less
  // than the BUTTON_THRESHOLD variable toward the top of the file, return true.
  // IMPORTANT: This function assumes pull up resistors
  if (getPinVoltage(button_pin) > BUTTON_THRESHOLD){
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
bool isCollision() {
  int sonar_distance = sonar.ping_cm(); // If the distance is too big, it returns 0.
  Serial.println(sonar_distance);
  if(sonar_distance != 0){ 
    return (sonar_distance < COLLISION_DISTANCE);
  } else {
	  return false;
  }

}

////////////////////////////////////////////////////////////////////
// Function that detects if the capacitive sensor is being touched
////////////////////////////////////////////////////////////////////
bool isCapacitiveSensorTouched() {
  static CapacitiveSensor capacitive_sensor = CapacitiveSensor(CAP_SENSOR_SEND, CAP_SENSOR_RECEIVE);
  if (capacitive_sensor.capacitiveSensor(CAP_SENSOR_SAMPLES) > CAP_SENSOR_TAU_THRESHOLD){
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if light is present
////////////////////////////////////////////////////////////////////
bool isLight(int pin) {
  float light = getPinVoltage(pin);
  // Serial.println(light); // Use this line to test
  return (light > PHOTODIODE_LIGHT_THRESHOLD);
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is to the right or left,
// and steering the robot accordingly.
////////////////////////////////////////////////////////////////////
void fsmSteerRobot() {
  static int steerRobotState = 0;
  static unsigned long timerStartTime;
  //Serial.print(steerRobotState); Serial.print("\t"); //uncomment for debugging

  switch (steerRobotState) {
    case 0: //light is not detected
      ActionRobotDrive = DRIVE_STOP;
      
      //State transition logic
      if ( SensedLightLeft == DETECTION_YES ) {
        steerRobotState = 1; //if light on left of robot, go to left state
      } else if ( SensedLightRight == DETECTION_YES ) {
        steerRobotState = 2; //if light on right of robot, go to right state
      } else if ( SensedLight == DETECTION_NO ){
        steerRobotState = 4;
        timerStartTime = millis();
      }
      break;
    
    case 1: //light is to the left of robot
      //The light is on the left, turn left
      ActionRobotDrive = DRIVE_LEFT;
      
      //State transition logic
      if ( SensedLightRight == DETECTION_YES ) {
        steerRobotState = DRIVE_STRAIGHT; //if light is on right, then go straight
      } else if ( SensedLightLeft == DETECTION_NO ) {
        steerRobotState = DRIVE_STOP; //if light is not on left, go back to stop state
      }
      
      break;
    
    case 2: //light is to the right of robot
      //The light is on the right, turn right
      ActionRobotDrive = DRIVE_RIGHT;
      
      //State transition logic
      if ( SensedLightLeft == DETECTION_YES ) {
        steerRobotState = DRIVE_STRAIGHT; //if light is on left, then go straight
      } else if ( SensedLightRight == DETECTION_NO ) {
        steerRobotState = DRIVE_STOP; //if light is not on right, go back to stop state
      }

      break;
      
    case 3: // light is on both right and left
      //The light is straight ahead, go straight
      ActionRobotDrive = DRIVE_STRAIGHT;
      
      //State transition logic
      if ( SensedLightLeft == DETECTION_NO ) {
        steerRobotState = DRIVE_RIGHT; //if light is on right, then go right
      } else if ( SensedLightRight == DETECTION_NO ) {
        steerRobotState = DRIVE_LEFT; //if light is on left, then go left
      }
      
      break;

    case 4: // Search mode

      if(((elapsedTime(timerStartTime) / 1000) % 2) == 1){
        steerRobotState = DRIVE_RIGHT;
      } else {
        steerRobotState = DRIVE_LEFT;
      }

      //State transition logic
      if ( SensedLightLeft == DETECTION_YES ) {
        steerRobotState = 1; //if light on left of robot, go to left state
      } else if ( SensedLightRight == DETECTION_YES ) {
        steerRobotState = 2; //if light on right of robot, go to right state
      }

      break;
      
    default: // error handling
    {
      steerRobotState = 0;
    }
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is above or below center,
// and moving the servo accordingly.
////////////////////////////////////////////////////////////////////
void fsmMoveServoUpAndDown() {
  // Note that the a lower angle is towards the back of the robot
  static int moveServoState = 0;
  static unsigned long timerStartTime;
  //Serial.print(moveServoState); Serial.print("\t"); //uncomment for debugging
  
  // Milestone 3
  //Create a state machine modeled after the ones in milestones 1 and 2
  // to plan the servo action based off of the perception of the robot
  //Remember no light or light in front = servo doesn't move
  //Light above = servo moves up
  //Light below = servo moves down
  // Serial.print(moveServoState); Serial.print("\t"); //uncomment for debugging
  switch (moveServoState) {
    case 0: //light is not detected
      ActionServoMove = SERVO_MOVE_STOP;
      
      //State transition logic
      if ( SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_NO ) {
        moveServoState = 1; //if light above the robot, go to above state
      } else if ( SensedLightDown == DETECTION_YES && SensedLightUp == DETECTION_NO) {
        moveServoState = 2; //if light is below the robot, go to below state
      } else if ( SensedLight == DETECTION_NO ){
        moveServoState = 3;
        timerStartTime = millis();
      }
      break;
    
    case 1: //light is above the robot
      //The light is above the robot, move up
      ActionServoMove = SERVO_MOVE_UP;
      
      //State transition logic
      if ( (SensedLightDown == DETECTION_YES)  || (SensedLightUp == DETECTION_NO) ) {
        moveServoState = 0; //if light is above and below, or no longer visible don't move
      }

      break;
    
    case 2: //light is below the robot
      //The light is below, move down
      ActionServoMove = SERVO_MOVE_DOWN;
      
      //State transition logic
      if ( (SensedLightUp == DETECTION_YES)  || (SensedLightDown == DETECTION_NO) ) {
        moveServoState = 0; //if light is above and below, or no longer visible don't move
      }

      break;

    case 3: // Search mode
      if(((elapsedTime(timerStartTime) / 300) % 2) == 1 && ActionRobotSpeed != SPEED_STOP){
        ActionServoMove = SERVO_MOVE_DOWN;
      } else {
        ActionServoMove = SERVO_MOVE_UP;
      }

      //State transition logic
      if ( SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_NO ) {
        moveServoState = 1; //if light above the robot, go to above state
      } else if ( SensedLightDown == DETECTION_YES && SensedLightUp == DETECTION_NO) {
        moveServoState = 2; //if light is below the robot, go to below state
      }

      break;

    default: // error handling
    {
      moveServoState = 0;
    }
  }
}

////////////////////////////////////////////////////////////////////
// State machine for cycling through the robot's speeds.
////////////////////////////////////////////////////////////////////
void fsmChangeSpeed() {
  switch(ActionRobotSpeed){
    case SPEED_STOP:
      ActionRobotSpeed = SPEED_LOW;
      break;
    case SPEED_LOW:
      ActionRobotSpeed = SPEED_MED;
      break;
    case SPEED_MED:
      ActionRobotSpeed = SPEED_HIGH;
      break;
    case SPEED_HIGH:
      ActionRobotSpeed = SPEED_STOP;
      break;
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting when the capacitive sensor is
// touched, and changing the robot's speed.
////////////////////////////////////////////////////////////////////
void fsmCapacitiveSensorSpeedControl() {
  static bool sensor_touched = false;

  if (SensedCapacitiveTouch){
    sensor_touched = true;
  }

  if (sensor_touched && !SensedCapacitiveTouch){
    fsmChangeSpeed();
    sensor_touched = false;
  }
}

////////////////////////////////////////////////////////////////////
// Function that causes the servo to move up or down.
////////////////////////////////////////////////////////////////////
void MoveServo() {
    static int servoAngle = SERVO_START_ANGLE;
    // Serial.println(servoAngle);
    
    switch(ActionServoMove) {
    case SERVO_MOVE_UP: // servo moving in positive direction
      if(servoAngle >= SERVO_DOWN_LIMIT) {
        servoAngle--;
      }
      break;
    case SERVO_MOVE_DOWN: // servo moving in negative direction
      if(servoAngle <= SERVO_UP_LIMIT) {
        servoAngle++;
      }
      break;
    case SERVO_MOVE_STOP:
      break;
  }
  myServo.write(servoAngle); // send angle to the servo 
  // the .write() function expects an integer between 0 and 180 degrees
}

////////////////////////////////////////////////////////////////////
// Function that writes the pre-planned RGB values to the RGB LED.
////////////////////////////////////////////////////////////////////
void updateRgbLed(){
  digitalWrite(RED_PIN, rgbVals.redPwm);
  digitalWrite(GREEN_PIN, rgbVals.greenPwm);
  digitalWrite(BLUE_PIN, rgbVals.bluePwm);
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void RobotPerception() {
  // This function polls all of the sensors and then assigns sensor outputs
  // that can be used by the robot in subsequent stages
  
  // Photodiode Sensing
  //Serial.print(getPinVoltage(BUTTON_2)); Serial.print("\t"); //uncomment for debugging
  
  if (isLight(UPPER_INSIDE_PHOTODIODE) && isLight(BOTTOM_INSIDE_PHOTODIODE)){
    SensedLightLeft = DETECTION_YES;
  } else {
    SensedLightLeft = DETECTION_NO;
  }
  // Remember, you can find the buttons and which one goes to what towards the top of the file
  if (isLight(UPPER_OUTSIDE_PHOTODIODE) && isLight(BOTTOM_OUTSIDE_PHOTODIODE)) { 
    SensedLightRight = DETECTION_YES;
  } else {
    SensedLightRight = DETECTION_NO;
  }

      
  /* Add code to detect if light is up or down. Lab 2 milestone 3*/
  if (isLight(UPPER_INSIDE_PHOTODIODE) && isLight(UPPER_OUTSIDE_PHOTODIODE)){
    SensedLightUp = DETECTION_YES;
  } else {
    SensedLightUp = DETECTION_NO;
  }
  // Remember, you can find the buttons and which one goes to what towards the top of the file
  if (isLight(BOTTOM_INSIDE_PHOTODIODE) && isLight(BOTTOM_OUTSIDE_PHOTODIODE)) { 
    SensedLightDown = DETECTION_YES;
  } else {
    SensedLightDown = DETECTION_NO;
  }

  // Update the general SensedLight Variable
  if (SensedLightDown || SensedLightUp || SensedLightLeft || SensedLightRight){
    SensedLight = DETECTION_YES;
  } else {
    SensedLight = DETECTION_NO;
  }

  // Collision Sensor
  if (isCollision()) {   // Add code in isCollision() function for lab 2 milestone 1
    SensedCollision = DETECTION_YES;
  } else {
    SensedCollision = DETECTION_NO;
  }

  // Capacitive Sensor
  if (isCapacitiveSensorTouched()){
    SensedCapacitiveTouch = DETECTION_YES;
  } else {
    SensedCapacitiveTouch = DETECTION_NO;
  }
}

/**********************************************************************************************************
  Robot ACTION - implementing the decisions from planning to specific actions
 ********************************************************************/
void RobotAction() {
  // Here the results of planning are implented so the robot does something

  // This turns the collision LED on and off
  switch(ActionCollision) {
    case COLLISION_OFF:
      // doTurnLedOff(LED_3); //Collision LED off - DON'T FORGET TO ADD CODE TO doTurnLedOff() 
                           // AND doTurnLedOn() OR ELSE YOUR LEDS WON'T WORK!!!
      break;
    case COLLISION_ON:
      // doTurnLedOn(LED_3);
      // for(int i; i < (ActionRobotSpeed); i++){
      //   analogWrite(H_BRIDGE_ENA, 0);
      //   analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
      // }
      ActionRobotDrive = DRIVE_RIGHT;
      break;
  }
  
  // This drives the main motors on the robot
  switch(ActionRobotDrive) {
    case DRIVE_STOP:
      analogWrite(H_BRIDGE_ENA, 0);
      analogWrite(H_BRIDGE_ENB, 0);
      break;
    case DRIVE_STRAIGHT:
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
      break;
    case DRIVE_RIGHT:
      analogWrite(H_BRIDGE_ENA, 0);
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
      break;
    case DRIVE_LEFT:
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);
      analogWrite(H_BRIDGE_ENB, 0);
      break;
  }
  
  // This calls a function to move the servo
  MoveServo();
  
  // This calls a function to update the RGB LED
  updateRgbLed();
}


////////////////////////////////////////////////////////////////////
// State machine for detecting collisions, and stopping the robot
// if necessary.
////////////////////////////////////////////////////////////////////
void fsmCollisionDetection() {
  static int collisionDetectionState = 0;
  //Serial.print(collisionDetectionState); Serial.print("\t"); //uncomment for debugging
  
  switch (collisionDetectionState) {
    case 0: //collision detected
      //There is an obstacle, stop the robot
      ActionCollision = COLLISION_ON; // Sets the action to turn on the collision LED
      /* Add code in milestone 2 to stop the robot's wheels - Hint: ActionRobotDrive = ________ */
      
      //State transition logic
      if ( SensedCollision == DETECTION_NO) {
        collisionDetectionState = 1; //if no collision, go to no collision state
      }
      break;
    
    case 1: //no collision
      //There is no obstacle, drive the robot
      ActionCollision = COLLISION_OFF; // Sets action to turn off the collision LED

      fsmSteerRobot(); // Milestone 2
      
      //State transition logic
      if (isCollision()) {
        collisionDetectionState = 0; //if collision, go to collision state
      }
      break;

    default: // error handling
      {
        collisionDetectionState = 0;
      }
      break;
  }
}

void fsmRgbUpdate(){
  /*
    Updates in this function are structured as follows:
      sensing condition:
        bounds check (where applicable):
          LED PWM increment/decrement
    
    Bounds checks are necessary because if not present integer overflow
    will cause the LED to be continuously white.
  */

  // Check for collision
  if(SensedCollision){
    rgbVals.redPwm = 255;
  } else {
    if (rgbVals.redPwm > 0){
      rgbVals.redPwm--;
    }
  }

  // Check if the Capacitive Touch Sensor has been touched
  if(SensedCapacitiveTouch){
    if(rgbVals.bluePwm < 255){
      rgbVals.bluePwm++;
    }

    if(rgbVals.greenPwm > 0){
      rgbVals.greenPwm--;
    }

    if(rgbVals.redPwm > 0){
      rgbVals.redPwm--;
    }
  } else {
    if(rgbVals.bluePwm > 0){
      rgbVals.bluePwm++;
    }
  }

  // Check if light is detected
  if(SensedLight){
    if(rgbVals.greenPwm < 255){
      rgbVals.greenPwm--;
    }
  } else {
    if(rgbVals.greenPwm > 0){
      rgbVals.greenPwm--;
    }
  }
}


/**********************************************************************************************************
  Robot PLANNING - using the sensing to make decisions
 **********************************************************************************************************/
void RobotPlanning(void) {
  // The planning FSMs that are used by the robot to assign actions
  // based on the sensing from the Perception stage.
  fsmCollisionDetection(); // Milestone 1
  fsmMoveServoUpAndDown(); // Milestone 3
  fsmCapacitiveSensorSpeedControl(); // lab 4
  fsmRgbUpdate();
}


/********************************************************************
  Main LOOP function - this gets executed in an infinite loop until
  power off or reset. - Notice: PERCEPTION, PLANNING, ACTION
 ********************************************************************/
void loop() {
  // This DebugStateOutput flag can be used to easily turn on the
  // serial debugging to know what the robot is perceiving and what
  // actions the robot wants to take.
  int DebugStateOutput = false; // Change false to true to debug
  
  RobotPerception(); // PERCEPTION
  if (DebugStateOutput) {
    Serial.print("Perception:");
    Serial.print(SensedLightUp);
    Serial.print(SensedLightLeft);
    Serial.print(SensedCollision);
    Serial.print(SensedLightRight); 
    Serial.print(SensedLightDown);
    Serial.print(SensedCapacitiveTouch);
    Serial.print("\t");
    
    if (isLight(BOTTOM_OUTSIDE_PHOTODIODE)){
      Serial.println("BOTTOM_OUTSIDE_PHOTODIODE");
    }
    if (isLight(UPPER_INSIDE_PHOTODIODE)){
      Serial.println("UPPER_INSIDE_PHOTODIODE");
    }
    if (isLight(UPPER_OUTSIDE_PHOTODIODE)){
      Serial.println("UPPER_OUTSIDE_PHOTODIODE");
    }
    if (isLight(BOTTOM_INSIDE_PHOTODIODE)){
      Serial.println("BOTTOM_INSIDE_PHOTODIODE");
    }
  }
  
  RobotPlanning(); // PLANNING
  if (DebugStateOutput) {
    Serial.print(" Action:");
    Serial.print(ActionCollision);
    Serial.print(ActionRobotDrive); 
    Serial.print(ActionServoMove);
    Serial.print(" "); Serial.print(ActionRobotSpeed);
    Serial.print("\t");
  }
  RobotAction(); // ACTION
  Serial.print("\n");

  // float voltage = getPinVoltage(A1);

  // Serial.print("Voltage:\n");
  // Serial.print(voltage);
  // Serial.print('\n');

  // if(getPinVoltage(A1) > 4.0 ){
  //   doTurnLedOn(10);
  //   doTurnLedOn(11);
  //   doTurnLedOn(12);
  // } else if(getPinVoltage(A1) > 3.0 ){
  //   doTurnLedOff(12);
  //   doTurnLedOn(11);
  //   doTurnLedOn(10);
  // } else if (getPinVoltage(A1) > 2.0){
  //   doTurnLedOff(12);
  //   doTurnLedOff(11);
  //   doTurnLedOn(10);
  // } else {
  //   doTurnLedOff(10);
  //   doTurnLedOff(11);
  //   doTurnLedOff(12);
  // }
}

// // ================================== MOVE TO PARTICULAR ANGLE =========================================
// #include "Arduino.h"
// #include <Servo.h>  // loads the Servo library

// // Servo pin
// #define SERVO_PIN 10

// // Parameters for servo control as well as instantiation
// #define SERVO_START_ANGLE 0
// #define SERVO_UP_LIMIT 180
// #define SERVO_DOWN_LIMIT 0
// static Servo myServo;

// void setup() {
//   Serial.begin(9600);  // set up serial connection at 9600 Baud

//   //Set up servo
//   myServo.attach(SERVO_PIN);
//   myServo.write(SERVO_START_ANGLE);
// }
// void loop() { // We will replace this on the next slide
 
// }


// ============================== MOVE BACK AND FORTH ================================
// #include "Arduino.h"
// #include <Servo.h>  // loads the Servo library

// // Servo pin
// #define SERVO_PIN 10

// // Parameters for servo control as well as instantiation
// #define SERVO_START_ANGLE 90
// #define SERVO_UP_LIMIT 180
// #define SERVO_DOWN_LIMIT 0
// static Servo myServo;


// void MoveServo() {
//     static int state = 0;
//     static int servoAngle = SERVO_START_ANGLE;
//     Serial.println(servoAngle);
    
//     switch(state) {
//     case 0: // servo moving in positive direction
//       servoAngle++;
//       if(servoAngle >= SERVO_UP_LIMIT) {
//         state = 1;
//       }
//       break;
//     case 1: // servo moving in negative direction
//       servoAngle--;
//       if(servoAngle <= SERVO_DOWN_LIMIT) {
//         state = 0;
//       }
//       break;
//   }
//   myServo.write(servoAngle); // send angle to the servo 
//   // the .write() function expects an integer between 0 and 180 degrees
// }

// void setup() {
//   Serial.begin(9600);  // set up serial connection at 9600 Baud

//   //Set up servo
//   myServo.attach(SERVO_PIN);
//   myServo.write(SERVO_START_ANGLE);
// }

// void loop() { // Replace your old void loop with this
//   MoveServo(); // state machine to move servo back and forth
// }

// // ========================================= ULTRASONIC SENSOR =========================================

// #include "Arduino.h"
// #define TRIGGER_PIN 8
// #define ECHO_PIN 9

// long getDurationRaw() {
//   long duration;
//   // Clear the TRIGGER_PIN
//   digitalWrite(TRIGGER_PIN, LOW);
//   delayMicroseconds(2);

//   // Set the TRIGGER_PIN on HIGH state 
//   // for 10 micro seconds
//   digitalWrite(TRIGGER_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIGGER_PIN, LOW);
  
//   // Reads the ECHO_PIN, returns the sound 
//   //wave travel time in microseconds
//   duration = pulseIn(ECHO_PIN, HIGH, 10000);
//   return(duration);
// }

// float getDistanceRaw() {
//     float duration = (float)getDurationRaw();
//     // duration is time for sonar to travel to 
//     // object and back
//     duration = duration / 2;  
//     // divide by 2 for travel time to object 
//     float c = 343;  // speed of sound in m/s
//     c = c * 100 / 1e6;  
// // speed of sound in cm/microseconds
//     // Calculate the distance in centimeters
//     float distance = duration * c;
//     return(distance);
// }

// float getDistanceSmoothed() {
//     static float distanceSmoothed = getDistanceRaw();
//     float distance = getDistanceRaw();
//     float alpha = 0.9; // alpha-filter constant
//     if (distance != 0) {
//       // this is an example of a measurement gate:
//       // sensor returns a 0 when it times out 
//       // (i.e., no measurement) ignore those measurements
          
//       // alpha filter all good measurements     
//       distanceSmoothed = alpha*distanceSmoothed +(1-alpha)*distance;
//     }
//     return(distanceSmoothed);
// }

// void setup() {
//   Serial.begin(9600); // set up serial connection
//   pinMode(TRIGGER_PIN, OUTPUT);   
// // pulse sent out through TRIGGER_PIN    
//   pinMode(ECHO_PIN, INPUT); 
// 	// return signal read through ECHO_PIN
// }

// void loop() {
//   float distance = getDistanceSmoothed();
//   Serial.println(distance);  
// }
