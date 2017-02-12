/*
define of the pins
0 Serial RX
1 Serial TX
2 
3
4
5
6 BUZZER
7 right motor direction control line
8 left motor direction control line
9 right motor PWM control line
10 left motor PWM control line
11
12 ZUMO_BUTTON
13 Led on the side of the board
A0
A1 Battery pin
A2
A3
A4 I2C SDA
A5 I2C SCL
*/

// TODO 

#define nDEBUG

// include of the libs from the zumo
#include <String.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <ZumoBuzzer.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <srf02ser.h>

// #define BUZZER 6
// #define ZUMO_BUTTON 12
#define PIN_LED 13
#define PIN_BATTERY A1
#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5

// errors
#define ERROR_US 0b1
#define ERROR_SERIAL 0b10

//Ultrasound defines
#define US_SENSOR_ADRESS 0x00
#define US_THRESHOLD_DETECTION 30

// ground sensor
#define NUM_SENSORS 6
#define QTR_THRESHOLD  100 // microseconds
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        400
#define FORWARD_SPEED     400
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     800 // ms

// battery
#define BATT_MIN_THRESHOLD 550 // 550*1.5*5/1024 = 4V minimum for the batteries

// Motors and moves
#define ROBOT_SPEED 200
#define TIME_TO_FORGET 1000 // avoid losing target all the time

ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
srf02ser mUSSensor(US_SENSOR_ADRESS);
ZumoReflectanceSensorArray groundSensor(QTR_NO_EMITTER_PIN);

bool isSerialInitialized = false;
byte isAnError = 0x0;
bool batteryLow = false;
bool isAnEnemyInFrontOfMe = false;
bool presence = false;
bool presence_memory = false;
unsigned long lastSeen = 0;
unsigned int sensor_values[NUM_SENSORS];

// ---- MARIO
#define MELODY_LENGTH 95
unsigned char currentIdx = 0;

// These arrays take up a total of 285 bytes of RAM (out of a limit of 1k (ATmega168), 2k (ATmega328), or 2.5k(ATmega32U4))
unsigned char note[MELODY_LENGTH] = 
{
  NOTE_E(5), SILENT_NOTE, NOTE_E(5), SILENT_NOTE, NOTE_E(5), SILENT_NOTE, NOTE_C(5), NOTE_E(5),
  NOTE_G(5), SILENT_NOTE, NOTE_G(4), SILENT_NOTE,

  NOTE_C(5), NOTE_G(4), SILENT_NOTE, NOTE_E(4), NOTE_A(4), NOTE_B(4), NOTE_B_FLAT(4), NOTE_A(4), NOTE_G(4),
  NOTE_E(5), NOTE_G(5), NOTE_A(5), NOTE_F(5), NOTE_G(5), SILENT_NOTE, NOTE_E(5), NOTE_C(5), NOTE_D(5), NOTE_B(4),

  NOTE_C(5), NOTE_G(4), SILENT_NOTE, NOTE_E(4), NOTE_A(4), NOTE_B(4), NOTE_B_FLAT(4), NOTE_A(4), NOTE_G(4),
  NOTE_E(5), NOTE_G(5), NOTE_A(5), NOTE_F(5), NOTE_G(5), SILENT_NOTE, NOTE_E(5), NOTE_C(5), NOTE_D(5), NOTE_B(4),

  SILENT_NOTE, NOTE_G(5), NOTE_F_SHARP(5), NOTE_F(5), NOTE_D_SHARP(5), NOTE_E(5), SILENT_NOTE,
  NOTE_G_SHARP(4), NOTE_A(4), NOTE_C(5), SILENT_NOTE, NOTE_A(4), NOTE_C(5), NOTE_D(5),

  SILENT_NOTE, NOTE_G(5), NOTE_F_SHARP(5), NOTE_F(5), NOTE_D_SHARP(5), NOTE_E(5), SILENT_NOTE,
  NOTE_C(6), SILENT_NOTE, NOTE_C(6), SILENT_NOTE, NOTE_C(6),

  SILENT_NOTE, NOTE_G(5), NOTE_F_SHARP(5), NOTE_F(5), NOTE_D_SHARP(5), NOTE_E(5), SILENT_NOTE,
  NOTE_G_SHARP(4), NOTE_A(4), NOTE_C(5), SILENT_NOTE, NOTE_A(4), NOTE_C(5), NOTE_D(5),

  SILENT_NOTE, NOTE_E_FLAT(5), SILENT_NOTE, NOTE_D(5), NOTE_C(5)
};

unsigned int duration[MELODY_LENGTH] =
{
  100, 25, 125, 125, 125, 125, 125, 250, 250, 250, 250, 250,

  375, 125, 250, 375, 250, 250, 125, 250, 167, 167, 167, 250, 125, 125,
  125, 250, 125, 125, 375,

  375, 125, 250, 375, 250, 250, 125, 250, 167, 167, 167, 250, 125, 125,
  125, 250, 125, 125, 375,

  250, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 125, 125, 125,

  250, 125, 125, 125, 250, 125, 125, 200, 50, 100, 25, 500,

  250, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 125, 125, 125,

  250, 250, 125, 375, 500
};
// ---- !MARIO

// TODO : we should be using template here
void debug(String debugToPrint)
{
	#ifdef DEBUG
	if(!isSerialInitialized)
		Serial.begin(9600); 
	Serial.print(debugToPrint);
	#endif
	return;
}
void debugln(String debugToPrint)
{
	#ifdef DEBUG
	if(!isSerialInitialized)
		Serial.begin(9600); 
	Serial.println(debugToPrint);
	#endif
	return;
}
void debug(int debugToPrint)
{
	#ifdef DEBUG
	if(!isSerialInitialized)
		Serial.begin(9600); 
	Serial.print(debugToPrint);
	#endif
	return;
}
void debugln(int debugToPrint)
{
	#ifdef DEBUG
	if(!isSerialInitialized)
		Serial.begin(9600); 
	Serial.println(debugToPrint);
	#endif
	return;
}
void debug(byte debugToPrint)
{
	#ifdef DEBUG
	if(!isSerialInitialized)
		Serial.begin(9600); 
	Serial.print(debugToPrint);
	#endif
	return;
}
void debugln(byte debugToPrint)
{
	#ifdef DEBUG
	if(!isSerialInitialized)
		Serial.begin(9600); 
	Serial.println(debugToPrint);
	#endif
	return;
}


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


void checkAndTuneError()
{
	if(isAnError != 0x0)
	{
		debug("error(s) : ");
		debugln(isAnError);
		for (int i = 0; i < isAnError; i++)
		{
			delay(100);
			buzzer.playNote(NOTE_G(3), 80, 15);
		}
	}
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

void setup() {
	if(!isSerialInitialized)
		Serial.begin(9600); 
	delay(50);
	isSerialInitialized = true;

	mUSSensor.init();
	// this will be use for debug and for the ultrasound sensor
	debugln("start of the robot");

	if(!mUSSensor.checkCommunication())
		isAnError = isAnError|ERROR_US;
	checkAndTuneError();
	waitForButtonAndCountDown();
}

void loop() 
{
	// process functions
	mUSSensor.process();
	//checkBatterieStatus();
	//playMusic();




	// rest of the code


	groundSensor.read(sensor_values);

	if (sensor_values[0] < QTR_THRESHOLD)
	{
		// if leftmost sensor detects line, reverse and turn to the right
		motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
		delay(REVERSE_DURATION);
		motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
		delay(TURN_DURATION);
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		// TODO : get the delais out !!!
	}
	else if (sensor_values[5] < QTR_THRESHOLD)
	{
		// if rightmost sensor detects line, reverse and turn to the left
		motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
		delay(REVERSE_DURATION);
		motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
		delay(TURN_DURATION);
		motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		// TODO : get the delais out !!!
	}
	else
	{
	    // otherwise, do the job
	  
		int distance = mUSSensor.getLastResult();
		if(distance < 50)
		{
			presence = true;
			lastSeen = millis();
		}
		else
		{
			if(millis()-lastSeen > TIME_TO_FORGET)
				presence = false;
		}


		//Serial.println(presence);
		if(presence_memory != presence)
		{
			//if(checkForEnemy())
			if(presence)
			{
				digitalWrite(PIN_LED,HIGH);
			    motors.setLeftSpeed(-ROBOT_SPEED);
			    motors.setRightSpeed(-ROBOT_SPEED);
			}
			else
			{
				//digitalWrite(PIN_LED,LOW);
			    motors.setLeftSpeed(-ROBOT_SPEED);
			    motors.setRightSpeed(ROBOT_SPEED);
			}
		}

		presence_memory = presence;
	}
}

void playMusic()
{
	if (currentIdx < MELODY_LENGTH && !buzzer.isPlaying())
	{
		// play note at max volume
		buzzer.playNote(note[currentIdx], duration[currentIdx], 15);
		currentIdx++;
	}
}


