// srf02ser.cpp
#include "srf02ser.h"
#include "Arduino.h"

unsigned char sensorAddress;
int stateMachine;
bool errorFlag;
unsigned long timestampRequest;

enum stateStatus
{
	initialisation = 0,
	asking,
	rangeRequested,
	rangeReceived,
	errorTimeout
};

void anErrorHasAraise()
{
  errorFlag = true;
}
void sendCmd(unsigned char address,unsigned char cmd)
{
  Serial.write(address);//set the address of SRF02(factory default is 0)
  delayMicroseconds(100);//serial data is fixed at 9600,N,8,2,so we need some time to creat the sencond stop bit
  Serial.write(cmd);//send the command to SRF02
  delayMicroseconds(100);//serial data is fixed at 9600,N,8,2,so we need some time to creat the sencond stop bit
}

void askRange()
{
  sendCmd(sensorAddress,US_CMD_REAL_RANGE_AUTO);
}

void askVersion()
{
  sendCmd(sensorAddress,US_CMD_GET_SOFTWARE_VERSION);
}

srf02ser::srf02ser(unsigned char address)
{
	sensorAddress = address;
	lastResult = -1;
	errorFlag = false;
	stateMachine = 0;
  timestampRequest = 0;
}

void srf02ser::init()
{
	Serial.begin(9600);
}

bool srf02ser::checkCommunication()
{
  // send a request for the version, 
 askVersion();
 unsigned long timeRequestVersion = millis();
 while(millis()-timeRequestVersion < US_TIMEOUT)
 {
  if(Serial.available()>0)
  {
    //while(Serial.available()) 
    Serial.read();
    return true;
  }
 }
 
  return false;
}

void srf02ser::process()
{
	
	switch(stateMachine)
	{
		case initialisation:
			errorFlag = false;
			stateMachine = asking;
			break;

		case asking:
			askRange();
			timestampRequest = millis();
			stateMachine = rangeRequested;
			break;

		case rangeRequested:
			if(Serial.available() < 2) 
			{
				if(millis()-timestampRequest > US_TIMEOUT)
				{
					stateMachine = errorTimeout;
				}
				break;
			}
			else
			{
				int tempResult = 0;
				// read the received value
				tempResult = Serial.read()<<8;
				tempResult |= Serial.read();
				lastResult = tempResult;
				stateMachine = rangeReceived;
			}
			break;

		case rangeReceived:
			stateMachine = asking;
			break;

		case errorTimeout:
			if(millis()-timestampRequest > US_TIMEOUT_WAIT)
			{
				// flush
				while(Serial.available()) Serial.read();
				stateMachine = initialisation;
			}
			break;

		default:
			stateMachine = initialisation;
			break;
	}
	

}




