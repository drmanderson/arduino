#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int ledPin = 5;
int buttonApin = 4;
int buttonBpin = 3;
int pos1 = 0; 
int pos2 = 180;

byte leds = 0;

void setup() 
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonApin, INPUT_PULLUP);  
  pinMode(buttonBpin, INPUT_PULLUP);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object  
}

void loop() 
{
  if (digitalRead(buttonApin) == LOW)
  {
    digitalWrite(ledPin, HIGH);
    myservo.write(pos1);   
  }
  if (digitalRead(buttonBpin) == LOW)
  {
    digitalWrite(ledPin, LOW);
    myservo.write(pos2);   
  }
}
