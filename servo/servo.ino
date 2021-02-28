#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

void setup () {
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Starting");
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

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
  
    Serial.println("Moving to 10 degrees :");
    move_servo (pwm2, 0, 10);
    delay(500);
    Serial.println("Moving to 90 degrees :");
    move_servo (pwm2,0,90);
    delay (500);
    Serial.println("Moving to 170 degrees :");
    move_servo(pwm2,0,170);
    delay(2000);

}