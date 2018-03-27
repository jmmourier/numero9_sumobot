/*
define of the pins
0 Serial RX
1 Serial TX
2 led on array sensor
3 BUZZER
4 array sensor
5 array sensor
6 controlXshut TOF 1
7 right motor direction control line
8 left motor direction control line
9 right motor PWM control line
10 left motor PWM control line
11 array sensor
12 ZUMO_BUTTON
13 Led on the side of the board
A0 array sensor
A1 Battery pin
A2 array sensor
A3 array sensor
A4 I2C SDA
A5 I2C SCL
*/

/*
 * TODO : 
 * calibration of the TOF
 * add a debug mode
 * add error managment
 * add class in order to clean up the code
 * 
 */


#define DEBUG

// include of the libs from the zumo
#include <String.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <ZumoBuzzer.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <srf02ser.h>
#include <Wire.h>
#include <VL53L0X.h>

// definition of pins
// #define ZUMO_BUTTON 12
#define PIN_LED 13
#define PIN_BATTERY A1
#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5
#define ENABLE_TOF_1 6

// ground sensor
#define NUM_SENSORS 6
#define QTR_THRESHOLD  500 // microseconds
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        400
#define FORWARD_SPEED     400
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     400 // ms

// battery
#define BATT_MIN_THRESHOLD 550 // 550*1.5*5/1024 = 4V minimum for the batteries

// Motors and moves
#define ROBOT_SPEED 400
#define TIME_TO_FORGET 1000 // avoid losing target all the time

// states for state machine
#define INITIALISATION  0
#define MOVING_FORWARD  1
#define STOPING         2
#define MOVING_UNTIL_WALL 3  
#define BATTERY_LOW     4


// class definitions
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
ZumoReflectanceSensorArray groundSensor(QTR_NO_EMITTER_PIN);
VL53L0X TOF1;
VL53L0X TOF2;

// variables
bool batteryLow = false; 
unsigned int sensor_values[NUM_SENSORS];
int state; // for state machine in the loop
  

// when the button is pushed the robot makes some sounds and go
// blocking function 
void waitForButtonAndCountDown(int bipToDo = 3)
{
  button.waitForButton();
  // play audible countdown
  for (int i = 0; i < bipToDo; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
  delay(1000);
}

void checkBatterieStatus()
{
  int batteryTension = analogRead(PIN_BATTERY);
  if(batteryTension < BATT_MIN_THRESHOLD)
  {
    batteryLow = true;
    buzzer.playNote(NOTE_C(2), 100, 10);
  }
  else
  {
    batteryLow = false;
  }
}

void initTOFSensors()
{
  pinMode(ENABLE_TOF_1,OUTPUT);
  digitalWrite(ENABLE_TOF_1,LOW);
  TOF2.init();
  TOF2.setTimeout(500);
  TOF2.setAddress(0x30);
  delay(300);
  digitalWrite(ENABLE_TOF_1,HIGH);
  TOF1.init();
  TOF1.setTimeout(500);
  TOF1.setAddress(0x30);
  delay(300);
  TOF1.startContinuous();
  TOF2.startContinuous();
}

bool obstacleInFront()
{
  // return true if obstacle from the TOF sensors
  int leftValue = TOF1.readRangeContinuousMillimeters();
  int rightValue = TOF2.readRangeContinuousMillimeters();
  
  Serial.print(leftValue);
  Serial.print("   ");
  Serial.print(rightValue);
  Serial.print("   diff ");
  Serial.println(leftValue - rightValue);
  
  if(leftValue > 140 && rightValue > 140)
  {
    return false;
  }
  return true;
}

void setup() {

  // init communications
	Serial.begin(9600); 
	delay(50);
	Wire.begin();
  delay(50);

  // init variables
  state = 0;

  // init modules
    // init Tof sensors
  initTOFSensors();
  
  // informing init is done
  buzzer.playNote(NOTE_G(6), 70, 15);
  delay(100);
  buzzer.playNote(NOTE_G(6), 70, 15);
  delay(100);
  buzzer.playNote(NOTE_G(6), 70, 15);
  delay(1000);
    
  // init done, ready for starting 
	waitForButtonAndCountDown();
  
}

void loop() 
{ 

  // looping on state machine
  switch(state)
  {
  case INITIALISATION :
    state = MOVING_UNTIL_WALL;
    break;
    
  case MOVING_FORWARD :
    motors.setSpeeds(-FORWARD_SPEED, -FORWARD_SPEED);
    break;
    
  case STOPING :
    motors.setSpeeds(0, 0);  
    break;
    
  case MOVING_UNTIL_WALL :
    if(obstacleInFront())
    {
      motors.setSpeeds(0, 0);  
    }
    else
    {
      motors.setSpeeds(-FORWARD_SPEED, -FORWARD_SPEED);  
    }
    
    break;
    
  case BATTERY_LOW :
    if(batteryLow == false )
    {
      // back to the initialisation mode
      state = INITIALISATION;
    }
    else
    {
      motors.setSpeeds(0, 0);
      buzzer.playNote(NOTE_G(3), 200, 15);
      delay(200);
    }
    break;
  
  }

// reading orders
  if(Serial.available()>0)
  {
    byte data = Serial.read();
    switch(data)
    {
      case 'A':
        buzzer.playNote(NOTE_G(3), 200, 15);
        delay(200);
        state = MOVING_UNTIL_WALL;
        break;
      case 'B':
        buzzer.playNote(NOTE_G(4), 200, 15);
        delay(200);
        state = STOPING;
        break;
      default : 
        buzzer.playNote(NOTE_G(1), 200, 15);
        delay(200);
        break;
    }
  }
// sending status 


// reading inputs
  // reading TOFs

  // reading ground brigthness

  // reading battery
  checkBatterieStatus();

  // reading button
  if(button.isPressed())
  {
    if(state==STOPING)
    {
      state = INITIALISATION;
    }
    else
    {
      state = STOPING;
    }
    delay(200); // antibounce
  }
}



