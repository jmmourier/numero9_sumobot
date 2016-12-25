#include <ZumoBuzzer.h>
#include <Pushbutton.h>

ZumoBuzzer buzzer;
Pushbutton button(ZUMO_BUTTON);

#define batteryPin A1

void setup()
{
	// init Serial
	Serial.begin(9600);
	Serial.print("Battery Level : ");
	Serial.println(analogRead(batteryPin)* 50000L * 3/2 / 1023,DEC);
}

void loop()
{
	button.waitForButton();
	ringBattLevel();
	showBattLevel();

}
int readBatt()
{
	return analogRead(batteryPin);
}

void ringBattLevel()
{
	//playNote(unsigned char note, unsigned int duration, unsigned char volume);
    buzzer.playNote(NOTE_A(5), 200, 15);
    delay(200);
    buzzer.playNote(NOTE_B(5), 200, 15);
	delay(200);
	buzzer.playNote(NOTE_A(5), 200, 15);
	delay(200);
	button.waitForButton();
}

void showBattLevel()
{

}