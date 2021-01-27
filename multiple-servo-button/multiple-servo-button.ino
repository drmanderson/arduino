
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


/* Constants */

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

const int RledPin = 12;
const int GledPin = 11;
const int buttonApin = 2;
const int buttonBpin = 3;
const int buttonCpin = 4;
const int buttonDpin = 5;

const int pos0 = 0;
const int pos1 = 90; 
const int pos2 = 180;

const int buttonInterval = 300; // Time between button checks


/* Variables */

unsigned long currentMillis = 0; 
unsigned long previousOnButtonMillis = 0; // time when button press last checked
unsigned long previousOffButtonMillis = 0; // time when button press last checked
byte buttonGreenLed_State = LOW;
byte buttonRedLed_State = LOW;
int servonumber = 0;
int angle = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("Button test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(RledPin, OUTPUT);
  pinMode(GledPin, OUTPUT);
  pinMode(buttonApin, INPUT_PULLUP);  
  pinMode(buttonBpin, INPUT_PULLUP);  
  pinMode(buttonCpin, INPUT_PULLUP);
  pinMode(buttonDpin, INPUT_PULLUP);
}

// the code inside loop() has been updated by Robojax
void loop() {
  
  readOnButtons(); 
  readOffButtons();
  switchLeds();

}

void readOnButtons() {

    if (digitalRead(buttonApin) == LOW || digitalRead(buttonCpin) == LOW )  {     
      buttonGreenLed_State = HIGH; 
      buttonRedLed_State = LOW;
      servonumber=1;
      angle=105;   
      move_servo();                                          
    }

}

void readOffButtons() {

    if (digitalRead(buttonBpin) == LOW || digitalRead(buttonDpin) == LOW )  {
      buttonRedLed_State = HIGH;
      buttonGreenLed_State = LOW;
      servonumber=1;
      angle=75;
      move_servo();
    }
}


void switchLeds() {

 digitalWrite(RledPin, buttonRedLed_State);
 digitalWrite(GledPin, buttonGreenLed_State);

}




int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}


void move_servo ()  {

  pwm.setPWM(servonumber, 0, angleToPulse(angle));
  Serial.print("moving servo ");
  Serial.print(servonumber);
  Serial.print(" to angle ");
  Serial.println(angle);

}
