// include the PWMServo library
#include <Adafruit_PWMServoDriver.h>

// Setup pwm objects and their addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);

#define NUMSERVOS   5 // Enter the number of servos being controlled
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

// Setup servio struct that holds pwm card, socket number and open/close angles.
struct ServoData {
  Adafruit_PWMServoDriver  pwmcard;        // PCA9685 card number
  byte  pwmcard_socket; // Socket on PCA9685 card
  byte  openangle;      // User Configurable servo angle for open point
  byte  closeangle;     // User Configurable servo angle for close point
};
ServoData servo[NUMSERVOS];

/* 74hc165 Width of pulse to trigger the shift register to read and latch.
*/
#define PULSE_WIDTH_USEC   5

/* 74hc165 Optional delay between shift register reads.
*/
#define POLL_DELAY_MSEC   20

int clockEnablePin  = 4;  // Connects to Clock Enable pin the 165 (15)
int dataPin         = 5;  // Connects to the Q7 pin the 165 (9)
int clockPin        = 6;  // Connects to the Clock pin the 165 (2)
int ploadPin        = 7;  // Connects to Parallel load pin the 165 (1)
int latchPinOut     = 9;  // ST_CP pin 12
int clockPinOut     = 10;  // SH_CP pin 11
int dataPinOut      = 11;  // DS pin 14
byte old_incoming1;    // old values for first 74HC165 chip
byte incoming1;        // new values for first 74HC165 chip
byte old_incoming2;    // old values for second 74HC165 chip
byte incoming2;        // new values for first 74HC165 chip
int LEDpattern1;        // LED Pattern to send to the first 74hc595s
int LEDpattern2;        // LED Pattern to send to the second 74hc595s
int LEDArray[9] = {B00000001, B00000010, B00000100, B00001000, B00010000, B00100000,B01000000,B10000000,B00000000};

void setup()
{
 
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Starting");

  // Setup 74HC165 Serial connections
  pinMode(ploadPin, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  // Setup 74HC696 serial connections
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);
  // Initialise first 74HC165 values.
  // For one chip only use incoming1.
  // For two chips use incoming1 and incoming2
  // For more 75HC165 chips added new variables as required up to incoming4.
  read_values();
  old_incoming1 = incoming1;
  old_incoming2 = incoming2;

  // initialise the 74HC595 with all LED off B00000000
  LEDpattern1 = B00000000;
  LEDpattern2 = B00000000;
  set_LEDS ();
  delay(1000);
  
  // Setup PWM PCA9685 cards
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // Setup NUMSERVOS servos
  // Remember to update ths number when new servos are added.
  servo[0].pwmcard = pwm2;
  servo[0].pwmcard_socket = 0;
  servo[0].openangle = 115;
  servo[0].closeangle = 45;

  servo[1].pwmcard = pwm2;
  servo[1].pwmcard_socket = 4;
  servo[1].openangle = 145;
  servo[1].closeangle = 70;

  servo[2].pwmcard = pwm2;
  servo[2].pwmcard_socket = 1;
  servo[2].openangle = 60;
  servo[2].closeangle = 135; 

  servo[3].pwmcard = pwm2;
  servo[3].pwmcard_socket = 5;
  servo[3].openangle = 60;
  servo[3].closeangle = 134;
 
  servo[4].pwmcard = pwm2;
  servo[4].pwmcard_socket = 8;
  servo[4].openangle = 135;
  servo[4].closeangle = 60;
  
  // Start board in safe positions with all junctions "closed"
  Serial.println("Closing : 0");
  move_servo (servo[0].pwmcard, servo[0].pwmcard_socket, servo[0].closeangle);
  move_servo (servo[1].pwmcard, servo[1].pwmcard_socket, servo[1].closeangle);
  delay(1000);
  Serial.println("Closing : 1");
  move_servo (servo[2].pwmcard, servo[2].pwmcard_socket, servo[2].closeangle);
  move_servo (servo[3].pwmcard, servo[3].pwmcard_socket, servo[3].closeangle); 
  delay(1000);
  Serial.println("Closing : 0");
  move_servo (servo[4].pwmcard, servo[4].pwmcard_socket, servo[4].closeangle);
  delay(1000);
  // Flash all the LEDS
  LEDpattern1 = B11111111;
  LEDpattern2 = B11111111;  
  set_LEDS ();
  delay(1000);
  LEDpattern1 = B00000000;
  LEDpattern2 = B00000000;  
  set_LEDS ();  
}

void read_values() {
  // Write pulse to load pin
  digitalWrite(ploadPin, LOW);
  delayMicroseconds(PULSE_WIDTH_USEC);
  digitalWrite(ploadPin, HIGH);
  delayMicroseconds(PULSE_WIDTH_USEC);
 
  // Get data from 74HC165 chips
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockEnablePin, LOW);
  incoming1 = shiftIn(dataPin, clockPin, MSBFIRST);
  incoming2 = shiftIn(dataPin, clockPin, MSBFIRST);
  digitalWrite(clockEnablePin, HIGH);
}

void set_LEDS () {
  // Set the LEDS that are on / off 
  digitalWrite(latchPinOut, LOW);
  shiftOut (dataPinOut, clockPinOut, MSBFIRST, LEDpattern1 );
  shiftOut (dataPinOut, clockPinOut, MSBFIRST, LEDpattern2 ); 
  digitalWrite (latchPinOut, HIGH);
}

void move_points ( ) {
  // Print to serial monitor
  // Move either one or two servos based on which button has been pressed
  if (incoming1 > 0 ) {
    switch (incoming1) {
  
      case B00000001:
        // Button "1:" - close juction one
        // This is a cross over so throws two servos
        Serial.print("Buton 1.  74HC615 BIN value : ");
        Serial.println(incoming1, BIN);
        move_servo (servo[0].pwmcard, servo[0].pwmcard_socket, servo[0].closeangle);
        move_servo (servo[1].pwmcard, servo[1].pwmcard_socket, servo[1].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,0);
        set_LEDS(); 
        delay(500);
        break;

      case B00000010:
        // Button "2" - open juntion one
        // This is a cross over so throws two servos
        Serial.print("Buton 2.  74HC615 BIN value : ");
        Serial.println(incoming1, BIN);          
        move_servo (servo[0].pwmcard, servo[0].pwmcard_socket, servo[0].openangle);
        move_servo (servo[1].pwmcard, servo[1].pwmcard_socket, servo[1].openangle); 
        LEDpattern1 = LEDpattern1 | LEDArray[0];
        set_LEDS();       
        delay(500);    
        break;

      case B00000100:
        // Button "3" - close Juction two;
        // This is a cross over so throws two servos
        Serial.print("Buton 3.  74HC615 BIN value : ");
        Serial.println(incoming1, BIN);   
        move_servo (servo[2].pwmcard, servo[2].pwmcard_socket, servo[2].closeangle);
        move_servo (servo[3].pwmcard, servo[3].pwmcard_socket, servo[3].closeangle);  
        LEDpattern1 = bitClear(LEDpattern1,1);
        set_LEDS();   
        delay(500);      
        break;

      case B00001000:
        // Button "4" - open Juction two;
        // This is a cross over so throws two servos
        Serial.print("Buton 4.  74HC615 BIN value : ");
        Serial.println(incoming1, BIN);       
        move_servo (servo[2].pwmcard, servo[2].pwmcard_socket, servo[2].openangle);
        move_servo (servo[3].pwmcard, servo[3].pwmcard_socket, servo[3].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[1];
        set_LEDS();                 
        delay(500);   
        break;

      case B00010000:
        Serial.print("Five : ");
        Serial.println(incoming1, BIN);   
        break;
  
      case B00100000:
        Serial.print("Six : ");
        Serial.println(incoming1, BIN);   
        break;

      case B01000000:
        Serial.print("Seven : ");
        Serial.println(incoming1, BIN);   
        break;
      
      case B10000000:
        Serial.print("Eight : ");
        Serial.println(incoming1, BIN);   
        break;
        
      default:
        Serial.print ("Default: ");
        Serial.println(incoming1, BIN);
        break;    
    }
  } else if (incoming2 > 0 ) {
    switch (incoming2) {
      
      case B00000001:
        Serial.print("Nine : ");
        Serial.println(incoming2, BIN);   
        break;

      case B00000010:
        Serial.print("Ten : ");
        Serial.println(incoming2, BIN);   
        break;

      case B00000100:
        Serial.print("Eleven : ");
        Serial.println(incoming2, BIN);   
        break;

      case B00001000:
        Serial.print("Twelve : ");
        Serial.println(incoming2, BIN);   
        break;

      case B00010000:
        // Button "5:" - close juction three
        // This is a simple point - one servo
        Serial.print("Buton 5.  74HC615 BIN value : ");
        Serial.println(incoming2, BIN);
        move_servo (servo[4].pwmcard, servo[4].pwmcard_socket, servo[4].closeangle);
        LEDpattern2 = bitClear(LEDpattern2,6);
        set_LEDS();          
        delay(500);
        break;
  
      case B00100000:
        // Button "6:" - close juction three
        // This is a simple point - one servo
        Serial.print("Buton 6.  74HC615 BIN value : ");
        Serial.println(incoming2, BIN);
        move_servo (servo[4].pwmcard, servo[4].pwmcard_socket, servo[4].openangle);
        LEDpattern2 = LEDpattern2 | LEDArray[6];
        set_LEDS();         
        delay(500); 
        break;

      case B01000000:
        Serial.print("Fifteen : ");
        Serial.println(incoming2, BIN); 
        LEDpattern2 = bitClear(LEDpattern2,7);
        set_LEDS();    
        break;
      
      case B10000000:
        Serial.print("Sixteen : ");
        Serial.println(incoming2, BIN);
        LEDpattern2 = LEDpattern2 | LEDArray[7];
        set_LEDS();    
        break;
        
      default:
        Serial.print ("Default: ");
        Serial.println(incoming2, BIN);
        break;    
    }
  }

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

void loop()
{
  read_values();
  if (incoming1 != old_incoming1 || incoming2 != old_incoming2) {
    move_points();
    old_incoming1 = incoming1;
    old_incoming2 = incoming2;
    set_LEDS();
  }
  delay(200);
}
