// include the PWMServo library
#include <Adafruit_PWMServoDriver.h>

// Setup pwm objects and their addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x42);


#define NUMSERVOS   2 // Enter the number of servos being controlled
#define SERVOMIN  95 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  590 // this is the 'maximum' pulse length count (out of 4096)

// Setup servio struct that holds pwm card, socket number and open/close angles.
struct ServoData {
  Adafruit_PWMServoDriver  pwmcard;        // PCA9685 card number
  byte  pwmcard_socket; // Socket on PCA9685 card
  byte  openangle;      // User Configurable servo angle for open point
  byte  closeangle;     // User Configurable servo angle for close point
};
ServoData servo[NUMSERVOS];
int currangle[NUMSERVOS];

void setup()
{

  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Starting");

  // Setup PWM PCA9685 cards
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // Setup NUMSERVOS servos
  // Remember to update ths number when new servos are added.
  servo[0].pwmcard = pwm;
  servo[0].pwmcard_socket = 0;
  servo[0].openangle = 120;
  servo[0].closeangle = 60;
  servo[1].pwmcard = pwm;
  servo[1].pwmcard_socket = 1;
  servo[1].openangle = 120;
  servo[1].closeangle = 60;

  // Start board in safe positions with all junctions "closed"
  for (int i = 0; i <= NUMSERVOS - 1; i++) {
    move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, 90, 90);
    currangle[i] = 90;
    delay(100);
  }
}


int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
  return pulse;
}

void move_servo (Adafruit_PWMServoDriver pwmcard, int srv, int ang, int curr)  {
  Serial.print("Inside move_servo: ang and curr are ");
  Serial.print(ang);
  Serial.print(" : ");
  Serial.println(curr);
  
  if ( ang < 90 ) {
    int incr=curr;
    while (incr>=ang) {
      pwmcard.setPWM(srv, 0, angleToPulse(incr));
      incr--;
      delay(50);
    }
  }
  else if ( ang > 90 ) {
    int incr=curr;
    while (incr<=ang) {
      pwmcard.setPWM(srv, 0, angleToPulse(incr));
      incr++;
      delay(50);
    }
  }
  else if ( ang == 90 ) {
    pwmcard.setPWM(srv, 0, angleToPulse(ang));
    Serial.print("     Setting to 90: ");
    Serial.println(ang);
  }
  else {
    Serial.println("Crap");
  }
}

void loop()
{
  delay (4000);

  Serial.println("========= Main loop =========");
  for (int i = 0; i <= NUMSERVOS - 1; i++) {
    Serial.print(servo[i].openangle);
    Serial.println(" degrees");
    move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, servo[i].openangle, currangle[i]);
    currangle[i] = servo[i].openangle;
  }
  for (int i = 0; i <= NUMSERVOS - 1; i++) {
    Serial.print(servo[i].closeangle);
    Serial.println(" degrees");
    move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, servo[i].closeangle, currangle[i]);
    currangle[i] = servo[i].closeangle;
  }
}
