#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int ledPin = 5;
int buttonApin = 4;
int buttonBpin = 3;
int buttonCpin = 2;
int pos0 = 90;
int pos1 = 130; 
int pos2 = 50;

byte leds = 0;

void setup() 
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonApin, INPUT_PULLUP);  
  pinMode(buttonBpin, INPUT_PULLUP);
  pinMode(buttonCpin, INPUT_PULLUP);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object  
}

void loop() 
{
  if (digitalRead(buttonApin) == LOW) // take servo to pos1 and turn on LED
  {
    digitalWrite(ledPin, HIGH);
    myservo.write(pos1);   
  }
  if (digitalRead(buttonBpin) == LOW) // Take servo to pos2 and turn off LED
  {
    digitalWrite(ledPin, LOW);
    myservo.write(pos2);   
  }
  if (digitalRead(buttonCpin) == LOW) // "Reset" the servo and turn off LED 
  {
    digitalWrite(ledPin, LOW);
    myservo.write(pos0);   
  }
}
