#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
const int buttonApin = 7; // CLose Servo
const int buttonBpin = 8; // Open Servo
const int buttonCpin = 9; // Servo 90 degs

int buttonAState = LOW; 
int buttonBState = LOW; 
int buttonCState = LOW; 
int pwm2_servo = 15;
int pwm_servo = 0;
int pwm1_servo = 4;

long lastDebounceTimeA = 0;  // the last time the output pin was toggled
long lastDebounceTimeB = 0;  // the last time the output pin was toggled
long lastDebounceTimeC = 0;  // the last time the output pin was toggled

long debounceDelay = 300;    // the debounce time; increase if the output flickers

#define NUMSERVOS  3

struct ServoData {
  Adafruit_PWMServoDriver  pwmcard;        // PCA9685 card number
  byte  pwmcard_socket; // Socket on PCA9685 card
  byte  openangle;      // User Configurable servo angle for open point
  byte  closeangle;     // User Configurable servo angle for close point
};

ServoData servo[NUMSERVOS];



void setup () {
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Starting servo.ino");
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates  pwm2.begin();
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates  pinMode(buttonApin, INPUT_PULLUP);
  pinMode(buttonApin, INPUT_PULLUP);
  pinMode(buttonBpin, INPUT_PULLUP);
  pinMode(buttonCpin, INPUT_PULLUP);  

  servo[0].pwmcard = pwm;
  servo[0].pwmcard_socket = 0;
  servo[0].openangle = 135;
  servo[0].closeangle = 45;

  servo[1].pwmcard = pwm1;
  servo[1].pwmcard_socket = 0;
  servo[1].openangle = 135;
  servo[1].closeangle = 45;

  servo[2].pwmcard = pwm2;
  servo[2].pwmcard_socket = 0;
  servo[2].openangle = 135;
  servo[2].closeangle = 45;

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
  // Get state of buttons
  buttonAState = digitalRead(buttonApin);
  buttonBState = digitalRead(buttonBpin);
  buttonCState = digitalRead(buttonCpin);

  if ( (millis() - lastDebounceTimeA) > debounceDelay) {
    // If the buttonA has been pressed lets move the servo
    if  (buttonAState == HIGH)  {
      Serial.println("Button A pressed");
      Serial.println("Moving to 45 degrees:");
        for (int i = 0; i <= NUMSERVOS-1; i++) {
//          Serial.print("Closing: ");
//          Serial.println(i);
          move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, servo[i].closeangle);
          delay(500);
        }
      lastDebounceTimeA = millis();
    }
  } // Close if(A time buffer)

  if ( (millis() - lastDebounceTimeB) > debounceDelay) {
    // If the buttonB has been pressed lets move the servo
    if (buttonBState == HIGH)  {
      Serial.println("Button B pressed");
      Serial.println("Moving to 135 degrees :");
        for (int i = 0; i <= NUMSERVOS-1; i++) {
 //         Serial.print("Opening: ");
 //         Serial.println(i);
          move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, servo[i].openangle);
          delay(500);
        }
      lastDebounceTimeB = millis();
    }
  } // Close if(B time buffer)
  
  if ( (millis() - lastDebounceTimeC) > debounceDelay) {
    // If the buttonC has been pressed lets move the servo
    if (buttonCState == HIGH) {
      Serial.println("Button C pressed");
      Serial.println("Moving to 90 degrees :");
        for (int i = 0; i <= NUMSERVOS-1; i++) {
//          Serial.print("Centralling: ");
//          Serial.println(i);
          move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, 90);
          delay(500);
        }
      lastDebounceTimeC = millis();
    }
  } // Close if(C time buffer)

}//Close the void loop
