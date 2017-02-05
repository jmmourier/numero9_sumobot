#include <ZumoMotors.h>
#include <Pushbutton.h>

/*
 * This example uses tùhe ZumoMotors library to drive each motor on the Zumo
 * forward, then backward. The yellow user LED is on when a motor should be
 * running forward and off when a motor should be running backward. If a
 * motor on your Zumo has been flipped, you can correct its direction by
 * uncommenting the call to flipLeftMotor() or flipRightMotor() in the setup()
 * function.
 */

#define LED_PIN 13
#define BUTT_PIN 12


Pushbutton button(ZUMO_BUTTON);
const int ROBOT_SPEED = 200;

ZumoMotors motors;
unsigned int reading;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  // serial for the Ultrasound
  Serial.begin(9600);
  while(!button.isPressed());
}

void loop()
{

  // run left motor forward
  
  SendCmd(0x00,0x51);//Real Ranging Mode - Result in centimeters
  delay(70);//time for SRF02 to measure the range
  SendCmd(0x00,0x5E);//Get Range, returns two bytes (high byte first) from the most recent ranging.
  delay(10);//wait for some time,let the Arduino receive 2 bytes data from the TX pin of SRF02
  if(Serial.available()>=2)//if two bytes were received
  {
    reading = Serial.read()<<8;//receive high byte (overwrites previous reading) and shift high byte to be high 8 bits
    reading |= Serial.read(); // receive low byte as lower 8 bits
  }
    if(reading<30)
    {
      digitalWrite(13,HIGH);
      // on avance
    motors.setLeftSpeed(-ROBOT_SPEED);
    motors.setRightSpeed(-ROBOT_SPEED);
    delay(2);
    }
    else
    {
      digitalWrite(13,LOW);
      // on tourne

    motors.setLeftSpeed(-ROBOT_SPEED);
    motors.setRightSpeed(ROBOT_SPEED);
    delay(2);
    }
}



void SendCmd(unsigned char address,unsigned char cmd)
{
  Serial.write(address);//set the address of SRF02(factory default is 0)
  delayMicroseconds(100);//serial data is fixed at 9600,N,8,2,so we need some time to creat the sencond stop bit
  Serial.write(cmd);//send the command to SRF02
  delayMicroseconds(100);//serial data is fixed at 9600,N,8,2,so we need some time to creat the sencond stop bit
}
