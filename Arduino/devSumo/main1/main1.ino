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

// #define BUZZER 6
// #define ZUMO_BUTTON 12
#define LED 13
#define BATTERY A1
#define I2C_SDA A4
#define I2C_SCL A5
#define US_SENSOR_ADRESS 0x00

// errors
#define ERROR_US 0b1
#define ERROR_SERIAL 0b10

//Ultrasound defines
#define TIME_OUT_TEST_US 70
#define US_CMD_GET_SOFTWARE_VERSION 0x5D

// CONST

ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

bool isSerialInitialized = false;
byte isAnError = 0x0;

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

// return true if the ultrasounf answer any byte
bool isUltrasoundResponding()
{
	bool result = false;
	// this function should not be called if serial is not initialised
	if(!isSerialInitialized)
	{
		isAnError = isAnError|ERROR_SERIAL;
		return false;
	}
	
	sendCmdToUS(US_SENSOR_ADRESS,US_CMD_GET_SOFTWARE_VERSION);
	unsigned long timeAsked = millis(); // for the timeout
	while(millis()-timeAsked < TIME_OUT_TEST_US)
	{
		if(Serial.available()>0)
		{
			byte version = Serial.read();
			return true;
		}
	}
	isAnError = isAnError|ERROR_US;
	return false;
}

// general function to send data to the ultrasound
void sendCmdToUS(unsigned char address,unsigned char cmd)
{
  Serial.write(address);//set the address of SRF02(factory default is 0)
  delayMicroseconds(100);//serial data is fixed at 9600,N,8,2,so we need some time to creat the sencond stop bit
  Serial.write(cmd);//send the command to SRF02
  delayMicroseconds(100);//serial data is fixed at 9600,N,8,2,so we need some time to creat the sencond stop bit
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

void setup() {
	if(!isSerialInitialized)
		Serial.begin(9600); 
	delay(50);
	isSerialInitialized = true;
	// this will be use for debug and for the ultrasound sensor
	debugln("start of the robot");
	isUltrasoundResponding();
	checkAndTuneError();
	waitForButtonAndCountDown();
}

void loop() {

}


