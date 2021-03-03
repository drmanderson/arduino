
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4); 


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

// For each 74HC595 there needs to be a LEDpatternx up to 4
// For more 75HC165 chips added new variables as required up to incoming4.
// For multiple chips the loops in read_values, set_LEDS and the if case structure in move_points
// will have to be extened accordingly for the new chips (3 and 4).

int clockEnablePin  = 4;  // Connects to Clock Enable pin the 165 (15)
int dataPin         = 5;  // Connects to the Q7 pin the 165 (9)
int clockPin        = 6;  // Connects to the Clock pin the 165 (2)
int ploadPin        = 7;  // Connects to Parallel load pin the 165 (1)
int latchPinOut     = 9;  // Connects to the RCLK pin of the 595 (12)  ST_CP
int clockPinOut     = 10;  // Connects to the SRCLK ping of the 595 (11) SH_CP
int dataPinOut      = 11;  // Connects to the SER pin of the 595 (14) DS
byte old_incoming1;    // old values for first 74HC165 chip
byte incoming1;        // new values for first 74HC165 chip
//byte old_incoming2;    // old values for second 74HC165 chip
//byte incoming2;        // new values for first 74HC165 chip
int LEDpattern1;       // LED Pattern to send to the first 74hc595
int LEDpattern2;       // LED Pattern to send to the second 74hc595
int LEDArray[8] = {B00000001, B00000010, B00000100, B00001000, B00010000, B00100000,B01000000,B10000000};



void setup()
{
   // Setup i2c LCD
  lcd.init();
  lcd.backlight();
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Initiating ...");
  delay(2000);
  lcd.clear();
    
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Initiating ..");

  // Setup 74HC165 Serial connections
  pinMode(ploadPin, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  // Required initial states of these two pins according to the datasheet timing diagram
  digitalWrite(clockPin, HIGH);
  digitalWrite(ploadPin,HIGH);


  // Setup 74HC696 serial connections
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);
  // Initialise first 74HC165 values.
  // For one chip only use incoming1.
  // For two chips use incoming1 and incoming2 etc
  read_values();
  old_incoming1 = incoming1;
//  old_incoming2 = incoming2;

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
//  servo[0].pwmcard = pwm2;
//  servo[0].pwmcard_socket = 0;
//  servo[0].openangle = 115;
//  servo[0].closeangle = 45;
//
//  servo[1].pwmcard = pwm2;
//  servo[1].pwmcard_socket = 4;
//  servo[1].openangle = 145;
//  servo[1].closeangle = 70;
//
//  servo[2].pwmcard = pwm2;
//  servo[2].pwmcard_socket = 1;
//  servo[2].openangle = 60;
//  servo[2].closeangle = 135; 
//
//  servo[3].pwmcard = pwm2;
//  servo[3].pwmcard_socket = 5;
//  servo[3].openangle = 60;
//  servo[3].closeangle = 134;
// 
//  servo[4].pwmcard = pwm;
//  servo[4].pwmcard_socket = 8;
//  servo[4].openangle = 135;
//  servo[4].closeangle = 60;
//  
//  // Start board in safe positions with all junctions "closed"
//  for (int i = 0; i <= NUMSERVOS-1; i++) {
//    Serial.print("Closing :");
//    Serial.println(i);k
//    move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, servo[i].closeangle);
//    delay(500);
//  } 
//  // Flash all the LEDS
  LEDpattern1 = B11111111;
  LEDpattern2 = B11111111;  
  set_LEDS ();
  delay(1000);
  LEDpattern1 = B00000000;
  LEDpattern2 = B00000000;  
  set_LEDS ();  

  lcd.print("Ready  ..... ");
}

void read_values() {

  
  // Write pulse to load pin
  digitalWrite(ploadPin, LOW);
  delayMicroseconds(5);
  digitalWrite(ploadPin, HIGH);
  delayMicroseconds(5);
 
  // Get data from 74HC165 chips
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockEnablePin, LOW);
  incoming1 = shiftIn(dataPin, clockPin, MSBFIRST);
//  incoming2 = shiftIn(dataPin, clockPin, MSBFIRST); 
//   incoming2 = 0;
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
    Serial.println(incoming1);
    switch (incoming1) {
  
      case B00000001:
        lcd.clear();
        lcd.home();
        Serial.print("One : ");
        lcd.print("One : ");
        lcd.setCursor(0,1);
        Serial.println(incoming1, BIN);
        lcd.print(incoming1, BIN);
        break;

      case B00000010:
        lcd.clear();
        lcd.home();
        Serial.print("Two : ");
        lcd.print("Two : ");
        lcd.setCursor(0,1);
        Serial.println(incoming1, BIN);
        lcd.print(incoming1, BIN); 
        break;

      case B00000100:
        lcd.clear();
        lcd.home();
        Serial.print("Three : ");
        lcd.print("Three : ");
        lcd.setCursor(0,1);
        Serial.println(incoming1, BIN);
        lcd.print(incoming1, BIN);
        break;

      case B00001000:
        lcd.clear();
        lcd.home();
        Serial.print("Four : ");
        lcd.print("Four : ");
        lcd.setCursor(0,1);
        Serial.println(incoming1, BIN);
        lcd.print(incoming1, BIN);
        break;

      case B00010000:
        Serial.print("Five : ");
        Serial.println(incoming1, BIN);
        lcd.clear();
        lcd.print(incoming1, BIN);  
        break;
  
      case B00100000:
        Serial.print("Six : ");
        Serial.println(incoming1, BIN);
        lcd.clear();
        lcd.print(incoming1, BIN); 
        break;

      case B01000000:
        Serial.print("Seven : ");
        Serial.println(incoming1, BIN);
        lcd.clear();
        lcd.print(incoming1, BIN);  
        break;
      
      case B10000000:
        Serial.print("Eight : ");
        Serial.println(incoming1, BIN);
        lcd.clear();
        lcd.print(incoming1, BIN);  
        break;
        
      default:
        Serial.print ( "74HC165-1 Default: ");
        Serial.println(incoming1, BIN);
        lcd.clear();
        lcd.print(incoming1, BIN);
        break;    
    }
  // } else if (incoming2 > 0 ) {
  //   switch (incoming2) {
      
  //     case B00000001:
  //       Serial.print("Nine : ");
  //       Serial.println(incoming2, BIN);   
  //       break;

  //     case B00000010:
  //       Serial.print("Ten : ");
  //       Serial.println(incoming2, BIN);   
  //       break;

  //     case B00000100:
  //       Serial.print("Eleven : ");
  //       Serial.println(incoming2, BIN);   
  //       break;

  //     case B00001000:
  //       Serial.print("Twelve : ");
  //       Serial.println(incoming2, BIN);   
  //       break;

  //     case B00010000:
  //       Serial.print("Thirteen : ");
  //       Serial.println(incoming2, BIN);
  //       break;
  
  //     case B00100000:
  //       Serial.print("Fourteen : ");
  //       Serial.println(incoming2, BIN);
  //       break;

  //     case B01000000:
  //       Serial.print("Fifteen : ");
  //       Serial.println(incoming2, BIN);  
  //       break;
      
  //     case B10000000:
  //       Serial.print("Sixteen : ");
  //       Serial.println(incoming2, BIN);  
  //       break;
        
  //     default:
  //       Serial.print (" 74HC165-2 Default: ");
  //       Serial.println(incoming2, BIN);
  //       break;    
  //   }
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
  if (incoming1 != old_incoming1 ) {  
 // if (incoming1 != old_incoming1 || incoming2 != old_incoming2) {
    move_points();
    old_incoming1 = incoming1;
 //   old_incoming2 = incoming2;
 //   set_LEDS();
  }
  delay(200);
}
