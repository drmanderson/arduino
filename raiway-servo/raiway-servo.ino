#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

const int point_1_open_angle = 115;
const int point_1_close_angle = 60;
const int point_2_open_angle = 145;
const int point_2_close_angle = 70;
const int buttonApin = 6;
const int buttonBpin = 7;
const int buttonInterval = 300; // Time between button checks

int point_1 = 0;
int point_2 = 4;


void setup() {
  Serial.begin(9600);
  Serial.println("Button test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(buttonApin, INPUT_PULLUP);
  pinMode(buttonBpin, INPUT_PULLUP);
  move_servo(point_1, point_1_close_angle);
  move_servo(point_2, point_2_close_angle);
}

void loop() {
  readOnButtons();
  readOffButtons();

}

void readOnButtons() {

    if (digitalRead(buttonApin) == LOW  )  {
      move_servo(point_1,point_1_close_angle);
      move_servo(point_2,point_2_close_angle);
    }

}

void readOffButtons() {

    if (digitalRead(buttonBpin) == LOW )  {
      move_servo(point_1,point_1_open_angle);
      move_servo(point_2,point_2_open_angle);
    }
}

int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}



void move_servo (int srv,int ang)  {

  pwm.setPWM(srv, 0, angleToPulse(ang));
  Serial.print("moving servo ");
  Serial.print(srv);
  Serial.print(" to angle ");
  Serial.println(ang);

}
