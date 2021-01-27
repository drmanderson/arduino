
 /*
 * Original sourse: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * 
 * This is the Arduino code PAC6985 16 channel servo controller
 * watch the video for details (V1) and demo http://youtu.be/y8X9X10Tn1k
 *  This code is #1 for V2 Video Watch the video :https://youtu.be/bal2STaoQ1M
 *  I have got 3 codes as follow:
   #1-Arduino Code to run one by one all servos from 0 to 180°   
   #2-Arduino Code to control specific servos with specific angle
   #3-Arduino Code to run 2 or all servos at together
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
// Watch video V1 to understand the two lines below: http://youtu.be/y8X9X10Tn1k
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;


int RledPin = 12;
int GledPin = 13;

int buttonApin = 2;
int buttonBpin = 3;
int buttonCpin = 4;
int buttonDpin = 5;

int pos0 = 0;
int pos1 = 180; 
int pos2 = 90;


void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(RledPin, OUTPUT);
  pinMode(GledPin, OUTPUT);

  pinMode(buttonApin, INPUT_PULLUP);  
  pinMode(buttonBpin, INPUT_PULLUP);  
  pinMode(buttonCpin, INPUT_PULLUP);
  pinMode(buttonDpin, INPUT_PULLUP);
  //yield();
}

// the code inside loop() has been updated by Robojax
void loop() {

  move_servo(0);
   
}

void move_servo (int servonumber) {

  pwm.setPWM(servonumber, 0, angleToPulse(5));
  delay(1000);  
  pwm.setPWM(servonumber, 0, angleToPulse(180));
  delay(1000);  
  pwm.setPWM(servonumber, 0, angleToPulse(90));
  delay(1000);
  
  
}

/*
/* angleToPulse(int ang)
 * @brief gets angle in degree and returns the pulse width
 * @param "ang" is integer represending angle from 0 to 180
 * @return returns integer pulse width
 * Usage to use 65 degree: angleToPulse(65);
 * Written by Ahmad Shamshiri on Sep 17, 2019. 
 * in Ajax, Ontario, Canada
 * www.Robojax.com 
 */
 
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}
