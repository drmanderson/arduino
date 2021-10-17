#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
const int buttonApin = 7; // CLose Servo
const int buttonBpin = 8; // Open Servo
const int buttonCpin = 9; // Servo 90 degs

int buttonAState = LOW; 
int buttonBState = LOW; 
int buttonCState = LOW; 

long lastDebounceTimeA = 0;  // the last time the output pin was toggled
long lastDebounceTimeB = 0;  // the last time the output pin was toggled
long lastDebounceTimeC = 0;  // the last time the output pin was toggled

long debounceDelay = 50;    // the debounce time; increase if the output flickers

int servoClosed = -1;
int servoOpen = -1;
int servo90 = -1;

void setup () {
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Starting servo.ino");
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(buttonApin, INPUT_PULLUP);
  pinMode(buttonBpin, INPUT_PULLUP);
  pinMode(buttonCpin, INPUT_PULLUP);  

}

int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
   return pulse;
}

void move_servo (Adafruit_PWMServoDriver pwmcard, int srv,int ang)  {
  pwmcard.setPWM(srv, 0, angleToPulse(ang));
  Serial.print("moving servo ");
  Serial.print(srv);
  Serial.print(" to angle ");
  Serial.println(ang);
}

void loop () {

  buttonAState = digitalRead(buttonApin);
  buttonAState = digitalRead(buttonApin);
  buttonAState = digitalRead(buttonApin);

  if ( (millis() - lastDebounceTimeA) > debounceDelay) {
    // If the buttonA has been pressed lets move the servo
    if ( (buttonAState == LOW) && (servoClosed < 0 ) ) {
      Serial.println("Button A pressed");
      Serial.println("Moving to 10 degrees :");
      move_servo (pwm2, 0, 10); 
      servoClosed = -servoClosed;
      lastDebounceTimeA = millis;
    }
  } // Close if(A time buffer)

  if ( (millis() - lastDebounceTimeB) > debounceDelay) {
    // If the buttonB has been pressed lets move the servo
    if ( (buttonBState == LOW) && (servoOpen < 0 ) ) {
      Serial.println("Button B pressed");
      Serial.println("Moving to 170 degrees :");
      move_servo (pwm2, 0, 10); 
      servoOpen = -servoOpen;
      lastDebounceTimeB = millis();
    }
  } // Close if(B time buffer)
  
  if ( (millis() - lastDebounceTimeC) > debounceDelay) {
    // If the buttonC has been pressed lets move the servo
    if ( (buttonCState == LOW) && (servo90 < 0 ) ) {
      Serial.println("Button C pressed");
      Serial.println("Moving to 90 degrees :");
      move_servo (pwm2, 0, 10); 
      servo90 = -servo90;
      lastDebounceTimeC = millis();
    }
  } // Close if(C time buffer)

}//Close the void loop
