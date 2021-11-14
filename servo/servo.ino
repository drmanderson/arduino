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

#define NUMSERVOS  17

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
  servo[0].openangle = 115;
  servo[0].closeangle = 45;

  servo[1].pwmcard = pwm;
  servo[1].pwmcard_socket = 1;
  servo[1].openangle = 130;
  servo[1].closeangle = 60;

  servo[2].pwmcard = pwm;
  servo[2].pwmcard_socket = 2;
  servo[2].openangle = 145;
  servo[2].closeangle = 50;

  servo[3].pwmcard = pwm1;
  servo[3].pwmcard_socket = 1;
  servo[3].openangle = 70;
  servo[3].closeangle = 135;

  servo[4].pwmcard = pwm1;
  servo[4].pwmcard_socket = 2;
  servo[4].openangle = 70;
  servo[4].closeangle = 145;

  servo[5].pwmcard = pwm1;
  servo[5].pwmcard_socket = 7;
  servo[5].openangle = 120;
  servo[5].closeangle = 65;

  servo[6].pwmcard = pwm1;
  servo[6].pwmcard_socket = 8;
  servo[6].openangle = 120;
  servo[6].closeangle = 70;

  servo[7].pwmcard = pwm1;
  servo[7].pwmcard_socket = 9;
  servo[7].openangle = 70;
  servo[7].closeangle = 137;
  
  servo[8].pwmcard = pwm1;
  servo[8].pwmcard_socket = 4;
  servo[8].openangle = 70;
  servo[8].closeangle = 110;

  servo[9].pwmcard = pwm1;
  servo[9].pwmcard_socket = 3;
  servo[9].openangle = 130;
  servo[9].closeangle = 70;

  servo[10].pwmcard = pwm1;
  servo[10].pwmcard_socket = 6;
  servo[10].openangle = 50;
  servo[10].closeangle = 110;

  servo[11].pwmcard = pwm1;
  servo[11].pwmcard_socket = 5;
  servo[11].openangle = 70;
  servo[11].closeangle = 135;

  servo[12].pwmcard = pwm1;
  servo[12].pwmcard_socket = 10;
  servo[12].openangle = 70;
  servo[12].closeangle = 110;

  servo[13].pwmcard = pwm1;
  servo[13].pwmcard_socket = 11;
  servo[13].openangle = 70;
  servo[13].closeangle = 120;

  servo[14].pwmcard = pwm1;
  servo[14].pwmcard_socket = 12;
  servo[14].openangle = 60;
  servo[14].closeangle = 140;
  
  servo[15].pwmcard = pwm1;
  servo[15].pwmcard_socket = 13;
  servo[15].openangle = 70;
  servo[15].closeangle = 140;

  servo[16].pwmcard = pwm2;
  servo[16].pwmcard_socket = 15;
  servo[16].closeangle = 50;
  servo[16].openangle = 120;
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
      Serial.println("Closing all:");
        for (int i = 0; i <= NUMSERVOS-1; i++) {
          Serial.print("Closing: ");
          Serial.println(i);
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
      Serial.println("Moving to 170 degrees :");
        for (int i = 0; i <= NUMSERVOS-1; i++) {
          Serial.print("Opening: ");
          Serial.println(i);
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
          Serial.print("Centralling: ");
          Serial.println(i);
          move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, 90);
          delay(500);
        }
      lastDebounceTimeC = millis();
    }
  } // Close if(C time buffer)

}//Close the void loop
