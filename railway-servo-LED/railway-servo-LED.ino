#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4); 

// include the PWMServo library
#include <Adafruit_PWMServoDriver.h>

// Setup pwm objects and their addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);

// Lets put our defines here.
#define NUMSERVOS  17 // Enter the number of servos being controlled
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
#define BYTES_VAL_T unsigned long
#define NUMBER_OF_CHIPS 4
#define DATA_WIDTH NUMBER_OF_CHIPS * 8


// Setup servo struct that holds pwm card, socket number and open/close angles.
struct ServoData {
  Adafruit_PWMServoDriver  pwmcard;        // PCA9685 card number
  byte  pwmcard_socket; // Socket on PCA9685 card
  byte  openangle;      // User Configurable servo angle for open point
  byte  closeangle;     // User Configurable servo angle for close point
};

ServoData servo[NUMSERVOS];

// For each 74HC595 there needs to be a LEDpatternx up to 4

const int clockEnablePin  = 4;  // Connects to Clock Enable pin the 165 (15)
const int dataPin         = 5;  // Connects to the Q7 pin the 165 (9)
const int clockPin        = 6;  // Connects to the Clock pin the 165 (2)
const int ploadPin        = 7;  // Connects to Parallel load pin the 165 (1)
const int latchPinOut     = 9;  // Connects to the RCLK pin of the 595 (12)  ST_CP
const int clockPinOut     = 10;  // Connects to the SRCLK ping of the 595 (11) SH_CP
const int dataPinOut      = 11;  // Connects to the SER pin of the 595 (14) DS

int LEDpattern1;       // LED Pattern to send to the first 74hc595
int LEDpattern2;       // LED Pattern to send to the second 74hc595
int LEDArray[8] = {B00000001, B00000010, B00000100, B00001000, B00010000, B00100000,B01000000,B10000000};

// Setup initial values
BYTES_VAL_T pinValues = 0;                    // new values for  74HC165 chips
BYTES_VAL_T oldPinValues = 0 ;                 // old values for  74HC165 chips

BYTES_VAL_T read_values() {

  BYTES_VAL_T bitVal;
  BYTES_VAL_T bytesVal = 0;

  // Write pulse to load pin
  digitalWrite(clockEnablePin, HIGH);
  digitalWrite(ploadPin, LOW);
  delayMicroseconds(5);
  digitalWrite(ploadPin, HIGH);
  digitalWrite(clockEnablePin, LOW);

  // Get data from 74HC165 chips
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  for (int i=0; i < DATA_WIDTH; i++) {
    // Read value in
    bitVal = digitalRead(dataPin);

    //Set the corresponding bit in bytesVal
    bytesVal |= (bitVal << (DATA_WIDTH-1 - i));

    //Pulse the Clock - rising edge shifts next bit in
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
  }
  return(bytesVal);
}

void setup()
{
  // Setup i2c LCD
  lcd.init();
  lcd.backlight();
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Setting up ....");
  delay(2000);
  lcd.clear();
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("railway-servo-LED....");
  Serial.println("Setting up ....");

  // Setup 74HC165 Serial connections
  pinMode(ploadPin, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  // Required initial states of these two pins according to the datasheet timing diagram
  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin,HIGH);

  // Setup 74HC696 serial connections
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);
  // Initialise first 74HC165 values.
  // For one chip only use incoming1.
  // For two chips use incoming1 and incoming2 etc

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

  // Setup NUMSERVOS servos
  // Remember to update ths number when new servos are added.
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
    servo[16].closeangle = 70;
    servo[16].openangle = 120;


// Start board in safe positions with all junctions "closed"
  for (int i = 0; i <= NUMSERVOS-1; i++) {
    Serial.print("Closing :");
    Serial.println(i);
    move_servo (servo[i].pwmcard, servo[i].pwmcard_socket, servo[i].closeangle);
    delay(500);
  }
  // Flash all the LEDS
  LEDpattern1 = B11111111;
  LEDpattern2 = B11111111;
  set_LEDS ();
  delay(1000);
  LEDpattern1 = B00000000;
  LEDpattern2 = B00000000;
  set_LEDS ();
	
  lcd.print("Setup done.");
  Serial.println("Setup done.");
}

void set_LEDS () {
  // Set the LEDS that are on / off
  digitalWrite(latchPinOut, LOW);
  shiftOut (dataPinOut, clockPinOut, MSBFIRST, LEDpattern1 );
  shiftOut (dataPinOut, clockPinOut, MSBFIRST, LEDpattern2 );
  digitalWrite (latchPinOut, HIGH);
}

void move_points ( int switchNum) {
  // Move either one or two servos based on which button has been pressed
  switch(switchNum){

      case 0:
        // Button "1:" - close juction A
        lcd.clear();
        lcd.home();
        Serial.print("Button 1.");
        move_servo (servo[0].pwmcard, servo[0].pwmcard_socket, servo[0].closeangle);
        move_servo (servo[1].pwmcard, servo[1].pwmcard_socket, servo[1].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,0);
        set_LEDS();
        lcd.print("Closing - one ");
        delay(500);
        break;

      case 1:
        // Button "2" - open juntion A
        lcd.clear();
        lcd.home();
        Serial.print("Button 2.");
        move_servo (servo[0].pwmcard, servo[0].pwmcard_socket, servo[0].openangle);
        move_servo (servo[1].pwmcard, servo[1].pwmcard_socket, servo[1].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[0];
        set_LEDS();
        lcd.print("Opening - one");
        delay(500);
        break;

      case 2:
        // Button "3" - close Juction B
        lcd.clear();
        lcd.home();
        Serial.print("Button 3.");
        move_servo (servo[2].pwmcard, servo[2].pwmcard_socket, servo[2].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,1);
        set_LEDS();
        lcd.print("Closing - B");
        delay(500);
        break;

      case 3:
        // Button "4" - open Juction B
        lcd.clear();
        lcd.home();
        Serial.print("Button 4.");
        move_servo (servo[2].pwmcard, servo[2].pwmcard_socket, servo[2].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[1];
        set_LEDS();
        lcd.print("Opening - B");
        delay(500);
        break;

      case 4:
        // Button "5" - open Juction C
        lcd.clear();
        lcd.home();
        Serial.print("Button 5.");
        move_servo (servo[3].pwmcard, servo[3].pwmcard_socket, servo[3].closeangle);
        move_servo (servo[4].pwmcard, servo[4].pwmcard_socket, servo[4].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,2);
        set_LEDS();
        lcd.print("Closing - C ");
        delay(500);
        break;

      case 5:
        // Button "6" - close Juction C
        lcd.clear();
        lcd.home();
        Serial.print("Button 6.");
        move_servo (servo[3].pwmcard, servo[3].pwmcard_socket, servo[3].openangle);
        move_servo (servo[4].pwmcard, servo[4].pwmcard_socket, servo[4].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[2];
        set_LEDS();
        lcd.print("Opening - C");
        delay(500);
        break;

      case 6:
        // Button "7" - close Juction D
        lcd.clear();
        lcd.home();
        Serial.print("Button 7.");
        move_servo (servo[5].pwmcard, servo[5].pwmcard_socket, servo[5].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,3);
        set_LEDS();
        lcd.print("Closing - D ");
        delay(500);
        break;

      case 7:
        // Button "8" - open Juction D
        lcd.clear();
        lcd.home();
        Serial.print("Button 8.");
        move_servo (servo[5].pwmcard, servo[5].pwmcard_socket, servo[5].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[3];
        set_LEDS();
        lcd.print("Opening - D");
        delay(500);
        break;

      case 8:
        // Button "9" - close Juction E
        lcd.clear();
        lcd.home();
        Serial.print("Button 9.");
        move_servo (servo[6].pwmcard, servo[6].pwmcard_socket, servo[6].closeangle);
        move_servo (servo[12].pwmcard, servo[12].pwmcard_socket, servo[12].closeangle) 
        
        ;
        
        LEDpattern1 = bitClear(LEDpattern1,4);
        set_LEDS();
        lcd.print("Closing - E");
        delay(500);
        break;

      case 9:
        // Button "10" - open Juction E
        lcd.clear();
        lcd.home();
        Serial.print("Button 10.");
        move_servo (servo[6].pwmcard, servo[6].pwmcard_socket, servo[6].openangle);
        move_servo (servo[12].pwmcard, servo[12].pwmcard_socket, servo[12].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[4];
        set_LEDS();
        lcd.print("Opening - E ");
        delay(500);
        break;

      case 10:
        // Button "11" - open Juction F
        lcd.clear();
        lcd.home();
        Serial.print("Buton 11.");
        move_servo (servo[7].pwmcard, servo[7].pwmcard_socket, servo[7].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[5];
        set_LEDS();
        lcd.print("Opening - six ");
        delay(500);
        break;

      case 11:
        // Button "12" - close Juction F
        lcd.clear();
        lcd.home();
        Serial.print("Buton 12.");
        move_servo (servo[7].pwmcard, servo[7].pwmcard_socket, servo[7].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,5);
        set_LEDS();
        lcd.print("Closing - six ");
        delay(500);
        break;

      case 12:
        // Button "13" - open Juction G
        lcd.clear();
        lcd.home();
        Serial.print("Buton 13.");
        move_servo (servo[8].pwmcard, servo[8].pwmcard_socket, servo[8].closeangle);
        LEDpattern1 = LEDpattern1 | LEDArray[6];
        set_LEDS();
        lcd.print("Opening - seven ");
        delay(500);
        break;

      case 13:
        // Button "14" - close Juction G
        lcd.clear();
        lcd.home();
        Serial.print("Buton 14.");
        move_servo (servo[8].pwmcard, servo[8].pwmcard_socket, servo[8].openangle);
        LEDpattern1 = bitClear(LEDpattern1,6);
        set_LEDS();
        lcd.print("Closing - seven ");
        delay(500);
        break;

      case 14:
        // Button "15" - open Juction H
        lcd.clear();
        lcd.home();
        Serial.print("Buton 15.");
        move_servo (servo[9].pwmcard, servo[9].pwmcard_socket, servo[9].openangle);
        LEDpattern2 = LEDpattern2 | LEDArray[7];
        set_LEDS();
        lcd.print("Opening - eight");
        delay(500);
        break;

      case 15:
        // Button "16" - close Juction H
        lcd.clear();
        lcd.home();
        Serial.print("Buton 16.");
        move_servo (servo[9].pwmcard, servo[9].pwmcard_socket, servo[9].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,7);
        set_LEDS();
        lcd.print("Closing - eight ");
        delay(500);
        break;

      case 16:
        // Button "17" - open Juction I
        lcd.clear();
        lcd.home();
        Serial.print("Buton 17.");
        move_servo (servo[10].pwmcard, servo[10].pwmcard_socket, servo[10].openangle);
        LEDpattern2 = LEDpattern2 | LEDArray[0];
        set_LEDS();
        lcd.print("Opening - nine");
        delay(500);
        break;

      case 17:
        // Button "18" - close Juction I
        lcd.clear();
        lcd.home();
        Serial.print("Buton 18.");
        move_servo (servo[10].pwmcard, servo[10].pwmcard_socket, servo[10].closeangle);
        LEDpattern2 = bitClear(LEDpattern2,0);
        set_LEDS();
        lcd.print("Closing - nine ");
        delay(500);
        break;

      case 18:
        // Button "19" - open Juction J
        lcd.clear();
        lcd.home();
        Serial.print("Buton 19.");
        move_servo (servo[11].pwmcard, servo[11].pwmcard_socket, servo[11].openangle);
        LEDpattern2 = LEDpattern2 | LEDArray[1];
        set_LEDS();
        lcd.print("Opening - ten ");
        delay(500);
        break;

      case 19:
        // Button "20" - close Juction J
        lcd.clear();
        lcd.home();
        Serial.print("Buton 20.");
        move_servo (servo[11].pwmcard, servo[11].pwmcard_socket, servo[11].closeangle);
        LEDpattern2 = bitClear(LEDpattern2,1);
        set_LEDS();
        lcd.print("Closing - ten ");
        delay(500);
        break;      

       case 20:
         // Button "21" - open Juction eleven
         lcd.clear();
         lcd.home();
         Serial.print("Buton 21.");
         move_servo (servo[13].pwmcard, servo[13].pwmcard_socket, servo[13].openangle);
         LEDpattern2 = LEDpattern2 | LEDArray[2];
         set_LEDS();
         lcd.print("Opening - eleven");
         delay(500);
         break;

       case 21:
         // Button "22" - close Juction eleven
         lcd.clear();
         lcd.home();
         Serial.print("Buton 22.");
         move_servo (servo[13].pwmcard, servo[13].pwmcard_socket, servo[13].closeangle);
         LEDpattern2 = bitClear(LEDpattern2,2);
         set_LEDS();
         lcd.print("Closing - eleven ");
         delay(500);
         break;      

      case 22:
         // Button "23" - open Juction twelve
         lcd.clear();
         lcd.home();
         Serial.print("Buton 23.");
         move_servo (servo[14].pwmcard, servo[14].pwmcard_socket, servo[14].openangle);
         LEDpattern2 = LEDpattern2 | LEDArray[3];
         set_LEDS();
         lcd.print("Opening - twelve");
         delay(500);
         break;

      case 23:
         // Button 24 - close Juction twelve
         lcd.clear();
         lcd.home();
         Serial.print("Buton 24.");
         move_servo (servo[14].pwmcard, servo[14].pwmcard_socket, servo[14].closeangle);
         LEDpattern2 = bitClear(LEDpattern2,3);
         set_LEDS();
         lcd.print("Closing - twelve");
         delay(500);
         break;   

      case 24:
        // Button 25 - close Juncton 13
        lcd.clear();
        lcd.home();
        Serial.print("Button 25.");
        move_servo (servo[15].pwmcard, servo[15].pwmcard_socket, servo[15].openangle);
        LEDpattern2 = bitClear(LEDpattern2,3);
        set_LEDS();
        lcd.print("Closing - thirteen");
        delay(500);
        break;         

      case 25:
        // Button 26 - close Juncton 13
        lcd.clear();
        lcd.home();
        Serial.print("Button 26.");
        move_servo (servo[15].pwmcard, servo[15].pwmcard_socket, servo[15].closeangle);
        LEDpattern2 = bitClear(LEDpattern2,3);
        set_LEDS();
        lcd.print("Opening - thirteen");
        delay(500);
        break;  

      case 26:
        // Button 27 - close Juncton 14
        lcd.clear();
        lcd.home();
        Serial.print("Button 27.");
        move_servo (servo[16].pwmcard, servo[16].pwmcard_socket, servo[16].openangle);
        LEDpattern2 = bitClear(LEDpattern2,3);
        set_LEDS();
        lcd.print("Closing - fourteen");
        delay(500);
        break;         

      case 27:
        // Button 28 - close Juncton 14
        lcd.clear();
        lcd.home();
        Serial.print("Button 28.");
        move_servo (servo[16].pwmcard, servo[16].pwmcard_socket, servo[16].closeangle);
        LEDpattern2 = bitClear(LEDpattern2,3);
        set_LEDS();
        lcd.print("Opening - fourteen");
        delay(500);
        break;  
      default:
         // default - it's broken
        lcd.clear();
        lcd.home();
        Serial.print ("Default: ");
        Serial.println(switchNum, BIN);
        lcd.print("Failure...");
        break;
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
  // Read all pin values
  pinValues = read_values();
  // Process any changes in the values and call to move_points
  if (pinValues != oldPinValues) {
    for (int i=0;i< DATA_WIDTH; i++){
      if ((pinValues >> i) & 1) {
        move_points(i);
      }
   }
    oldPinValues = pinValues;
    // Set LED to reflect new settings
    set_LEDS();
  }
  delay(200);
}
