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
#define PIN_LED 13
#define PIN_BATTERY A1
#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5

// errors
#define ERROR_US 0b1
#define ERROR_SERIAL 0b10

//Ultrasound defines
#define US_TIMEOUT_TEST 70
#define US_CMD_GET_SOFTWARE_VERSION 0x5D
#define US_CMD_REAL_RANGE 0x54
#define US_CMD_GET_RANGE 0x5E
#define US_SENSOR_ADRESS 0x00
#define US_THRESHOLD_DETECTION 30

// battery
#define BATT_MIN_THRESHOLD 1 // TODO : set a real thresohl

// Motors and moves
#define ROBOT_SPEED 200

// CONST

ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

bool isSerialInitialized = false;
byte isAnError = 0x0;
bool batteryLow = false;
unsigned long USTimeLastRequest = 0;
bool isUSWorking = false;
bool isAnEnemyInFrontOfMe = false;

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
	while(millis()-timeAsked < US_TIMEOUT_TEST)
	{
		if(Serial.available()>0)
		{
			byte version = Serial.read();
			isUSWorking = false;
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
  USTimeLastRequest = millis();
  isUSWorking = true;
}

int read2ByteUS()
{
	int result = -1;
	if(Serial.available()>=2)
	{
		result = Serial.read()<<8;
		result |= Serial.read();
		isUSWorking = false;
		return result;
	}
	else
	{
		return result;
	}
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
		batteryLow = true;
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

	// playing Mario
	//playMusic();

	// rest of the code
	// if(bordureIsDetected)
	// 		Manoeuvre d'esquive
	// else 
	// 		Manoeuvre d'attaque

	if(checkForEnemy())
	{
		digitalWrite(PIN_LED,HIGH);
	    motors.setLeftSpeed(-ROBOT_SPEED);
	    motors.setRightSpeed(-ROBOT_SPEED);
	}
	else
	{
		digitalWrite(PIN_LED,LOW);
	    motors.setLeftSpeed(-ROBOT_SPEED);
	    motors.setRightSpeed(ROBOT_SPEED);
	}

	if(isUSWorking)
		digitalWrite(PIN_LED,HIGH);
	else
		digitalWrite(PIN_LED,LOW);
}

bool checkForEnemy()
{
/*
	send command
	wait 70 ms
	send seconde command
	wait serial available
	read byte
	get distante
	set enemy presence
*/

	if(isUSWorking)
	{
		int resultUS = read2ByteUS();
		if(resultUS == -1)
			return isAnEnemyInFrontOfMe;
		else 
			{
				if(resultUS < US_THRESHOLD_DETECTION)
					isAnEnemyInFrontOfMe = true;
				else
					isAnEnemyInFrontOfMe = false;
			}
	}
	else
	{
		sendCmdToUS(US_SENSOR_ADRESS,US_CMD_REAL_RANGE);
		debugln("a");
	}
	return isAnEnemyInFrontOfMe;
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


