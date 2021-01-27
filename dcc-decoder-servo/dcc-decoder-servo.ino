#define NUMSERVOS   2 // Enter the number of servos here
#define SERVOSPEED 30 // [ms] between servo updates, lower is faster


// GO TO setup() TO CONFIGURE DCC ADDRESSES, PIN NUMBERS, SERVO ANGLES

#include <DCC_Decoder.h>
#include <Adafruit_PWMServoDriver.h>

#define NUMCARDS   3  // Number of PCA9585 cards on I2C bus
// Initialise the NUMCARDS and their addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);



#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)


typedef struct DCCAccessoryData {
  int   address;        // User Configurable DCC address
  byte  outputpin;
  Adafruit_PWMServoDriver  pwmcard;        // PCA9685 card number
  byte  pwmcard_socket; // Socket on PCA9685 card
  byte  dccstate;       // Internal use DCC state of accessory: 1=on, 0=off
  byte  angle;          // Internal use current angle of servo
  byte  setpoint;       // Internal use current angle of servo
  byte  openangle;      // User Configurable servo angle for DCC state = 0
  byte  closeangle;     // User Configurable servo angle for DCC state = 1
};


DCCAccessoryData servo[NUMSERVOS];

void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data) {
  address -= 1;
  address *= 4;
  address += 1;
  address += (data & 0x06) >> 1;
  boolean enable = (data & 0x01) ? 1 : 0;
  for (byte i = 0; i < NUMSERVOS; i++) {
    if (address == servo[i].address) {
      if (enable) servo[i].dccstate = 1;
      else servo[i].dccstate = 0;
    }
  }
}

void setup() {
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // CONFIGURATION OF SERVOS
  // Copy & Paste as many times as you have servos
  // The amount must be same as NUMSERVOS
  // Don't forget to increment the array index
  servo[0].address   =   1 ;      // DCC address
  servo[0].outputpin = 5;
  servo[0].pwmcard =  pwm ;       // pc9685 that servo is plugged into
  servo[0].pwmcard_socket = 0;    // Arduino servo pin
  servo[0].openangle  =  115 ;    // Servo angle for DCC state = 0
  servo[0].closeangle   = 60 ;    // Servo angle for DCC state = 1

  servo[1].address   =   2 ;      // DCC address
  servo[1].outputpin = 6;
  servo[1].pwmcard =  pwm ;      // pc9685 that servo is plugged into
  servo[1].pwmcard_socket = 4;    // Arduino servo pin
  servo[1].openangle  =  145 ;    // Servo angle for DCC state = 0
  servo[1].closeangle   = 70 ;    // Servo angle for DCC state = 1


  DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
  DCC.SetupDecoder( 0x00, 0x00, 0 );

  for (byte i = 0; i < NUMSERVOS; i++) {
    servo[i].pwmcard.setPWM(servo[i].pwmcard_socket, 0, map(servo[i].closeangle, 0, 180, SERVOMIN, SERVOMAX));
    delay(1000); // wait 1 second before activating the next servo
    pinMode (servo[i].outputpin, OUTPUT);
    digitalWrite(servo[1].outputpin, LOW);
  }
}

void loop() {
  DCC.loop(); // Call to library function that reads the DCC data
  
  for (byte i = 0; i < NUMSERVOS; i++) {  
    if (servo[i].dccstate){
      digitalWrite(servo[i].outputpin, HIGH);
      servo[i].pwmcard.setPWM(servo[i].pwmcard_socket, 0, map(servo[i].closeangle, 0, 180, SERVOMIN, SERVOMAX));
    }
    else {
      digitalWrite(servo[i].outputpin, LOW);
      servo[i].pwmcard.setPWM(servo[i].pwmcard_socket, 0, map(servo[i].openangle, 0, 180, SERVOMIN, SERVOMAX));
    }
  }
}
