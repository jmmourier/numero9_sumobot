#include <srf02ser.h>

#define ADRESS_US 0x00
#define PIN_LED 13

srf02ser mySRF02(ADRESS_US);

void setup()
{
	pinMode(PIN_LED,OUTPUT);
	mySRF02.init();
  bool isCommunicationOK = mySRF02.checkCommunication();
  // led is on if the communication is OK
  
  if(isCommunicationOK)
    digitalWrite(PIN_LED,HIGH);
  else
    digitalWrite(PIN_LED,LOW);

  delay(2000);
}

void loop()
{
	mySRF02.process();
	if(mySRF02.getLastResult() < 30)
		digitalWrite(PIN_LED,HIGH);
	else
		digitalWrite(PIN_LED,LOW);
}
