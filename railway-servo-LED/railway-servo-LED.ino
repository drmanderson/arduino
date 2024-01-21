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
#define NUMSERVOS 28 // Enter the number of servos being controlled
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

uint16_t LEDpattern1;       // LED Pattern to send to the first 74hc595
uint16_t LEDpattern2;       // LED Pattern to send to the second 74hc595
uint16_t LEDArray[8] = {0b0000000000000001,
                   0b0000000000000010,
                   0b0000000000000100,
                   0b0000000000001000,
                   0b0000000000010000,
                   0b0000000000100000,
                   0b0000000001000000,
                   0b0000000010000000};
uint16_t LEDArray2[8] = {0b0000000100000000,
                   0b0000001000000000,
                   0b0000010000000000,
                   0b0000100000000000,
                   0b0001000000000000,
                   0b0010000000000000,
                   0b0100000000000000,
                   0b1000000000000000};

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
  pwm2.begin();
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates  

  // Setup NUMSERVOS servos
  // Remember to update ths number when new servos are added.
    servo[0].pwmcard = pwm;
    servo[0].pwmcard_socket = 0;
    servo[0].openangle = 140;
    servo[0].closeangle = 65;

    servo[1].pwmcard = pwm;
    servo[1].pwmcard_socket = 1;
    servo[1].openangle = 110;
    servo[1].closeangle = 60;

    servo[2].pwmcard = pwm;
    servo[2].pwmcard_socket = 2;
    servo[2].openangle = 120;
    servo[2].closeangle = 60;

    servo[3].pwmcard = pwm;
    servo[3].pwmcard_socket = 3;
    servo[3].openangle = 125;
    servo[3].closeangle = 55;
    
    servo[16].pwmcard = pwm1;
    servo[16].pwmcard_socket = 0;
    servo[16].openangle = 130;
    servo[16].closeangle = 60;

    servo[17].pwmcard = pwm1;
    servo[17].pwmcard_socket = 1;
    servo[17].openangle = 70;
    servo[17].closeangle = 130;
    
    servo[18].pwmcard = pwm1;
    servo[18].pwmcard_socket = 2;
    servo[18].openangle = 60;
    servo[18].closeangle= 120;    

    servo[19].pwmcard = pwm1;
    servo[19].pwmcard_socket = 3;
    servo[19].openangle = 135;
    servo[19].closeangle= 60;    
   
    servo[20].pwmcard = pwm1;
    servo[20].pwmcard_socket = 4;
    servo[20].openangle = 120;
    servo[20].closeangle= 60;    

    servo[21].pwmcard = pwm1;
    servo[21].pwmcard_socket = 5;
    servo[21].openangle = 120;
    servo[21].closeangle= 60;  

    servo[22].pwmcard = pwm1;
    servo[22].pwmcard_socket = 6;
    servo[22].openangle = 65;
    servo[22].closeangle= 130;   
       
    servo[23].pwmcard = pwm1;
    servo[23].pwmcard_socket = 7;
    servo[23].openangle = 65;
    servo[23].closeangle= 135; 

    servo[24].pwmcard = pwm1;
    servo[24].pwmcard_socket = 8;
    servo[24].openangle = 130;
    servo[24].closeangle= 60; 

    servo[25].pwmcard = pwm1;
    servo[25].pwmcard_socket = 9;
    servo[25].openangle = 120;
    servo[25].closeangle= 70; 

    servo[26].pwmcard = pwm1;
    servo[26].pwmcard_socket = 10;
    servo[26].openangle = 140;
    servo[26].closeangle= 60; 

    servo[27].pwmcard = pwm1;
    servo[27].pwmcard_socket = 11;
    servo[27].openangle = 120;
    servo[27].closeangle= 60; 
  
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
        move_servo (servo[16].pwmcard, servo[16].pwmcard_socket, servo[16].closeangle);
        LEDpattern1 = bitClear(LEDpattern1,0);
        set_LEDS();
        lcd.print("Closing - A");
        delay(500);
        break;

      case 1:
        // Button "2" - open juntion A
        lcd.clear();
        lcd.home();
        Serial.print("Button 2.");
        move_servo (servo[16].pwmcard, servo[16].pwmcard_socket, servo[16].openangle);
        LEDpattern1 = LEDpattern1 | LEDArray[0];
        set_LEDS();
        lcd.print("Opening - A");
        delay(500);
        break;

      case 2:
        // Button "3" - close Juction B
        lcd.clear();
        lcd.home();
        Serial.print("Button 3.");
        move_servo (servo[17].pwmcard, servo[17].pwmcard_socket, servo[17].closeangle);
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
        move_servo (servo[17].pwmcard, servo[17].pwmcard_socket, servo[17].openangle);
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
        move_servo (servo[18].pwmcard, servo[18].pwmcard_socket, servo[18].closeangle);
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
        move_servo (servo[18].pwmcard, servo[18].pwmcard_socket, servo[18].openangle);
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
        move_servo (servo[2].pwmcard, servo[2].pwmcard_socket, servo[2].closeangle);
        move_servo (servo[19].pwmcard, servo[19].pwmcard_socket, servo[19].closeangle);
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
        move_servo (servo[2].pwmcard, servo[2].pwmcard_socket, servo[2].openangle);
        move_servo (servo[19].pwmcard, servo[19].pwmcard_socket, servo[19].openangle);      
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
        move_servo (servo[20].pwmcard, servo[20].pwmcard_socket, servo[20].closeangle);
        move_servo (servo[21].pwmcard, servo[21].pwmcard_socket, servo[21].openangle);
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
        move_servo (servo[20].pwmcard, servo[20].pwmcard_socket, servo[20].openangle);
        move_servo (servo[21].pwmcard, servo[21].pwmcard_socket, servo[21].closeangle);
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
        move_servo (servo[22].pwmcard, servo[22].pwmcard_socket, servo[22].openangle);
        move_servo (servo[23].pwmcard, servo[23].pwmcard_socket, servo[23].openangle);
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
        move_servo (servo[22].pwmcard, servo[22].pwmcard_socket, servo[22].closeangle);
        move_servo (servo[23].pwmcard, servo[23].pwmcard_socket, servo[23].closeangle);
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
        move_servo (servo[24].pwmcard, servo[24].pwmcard_socket, servo[24].closeangle);
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
        move_servo (servo[24].pwmcard, servo[24].pwmcard_socket, servo[24].openangle);
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
        move_servo (servo[25].pwmcard, servo[25].pwmcard_socket, servo[25].openangle);
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
        move_servo (servo[25].pwmcard, servo[25].pwmcard_socket, servo[25].closeangle);
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
        move_servo (servo[26].pwmcard, servo[26].pwmcard_socket, servo[26].openangle);
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
        move_servo (servo[26].pwmcard, servo[26].pwmcard_socket, servo[26].closeangle);
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
        move_servo (servo[27].pwmcard, servo[27].pwmcard_socket, servo[27].openangle);
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
        move_servo (servo[27].pwmcard, servo[27].pwmcard_socket, servo[27].closeangle);
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
         move_servo (servo[0].pwmcard, servo[0].pwmcard_socket, servo[0].openangle);
         move_servo (servo[1].pwmcard, servo[1].pwmcard_socket, servo[1].openangle);
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
         move_servo (servo[0].pwmcard, servo[0].pwmcard_socket, servo[0].closeangle);
         move_servo (servo[1].pwmcard, servo[1].pwmcard_socket, servo[1].closeangle);
         LEDpattern2 = bitClear(LEDpattern2,2);
         set_LEDS();
         lcd.print("Closing - eleven ");
         delay(500);
         break;      

       case 22:
         // Button "21" - open Juction twelve
         lcd.clear();
         lcd.home();
         Serial.print("Buton 22.");
         move_servo (servo[3].pwmcard, servo[3].pwmcard_socket, servo[3].openangle);
         LEDpattern2 = LEDpattern2 | LEDArray[2];
         set_LEDS();
         lcd.print("Opening - twelve");
         delay(500);
         break;

       case 23:
         // Button "23" - close Juction twelve
         lcd.clear();
         lcd.home();
         Serial.print("Buton 23.");
         move_servo (servo[3].pwmcard, servo[3].pwmcard_socket, servo[3].closeangle);
         LEDpattern2 = bitClear(LEDpattern2,2);
         set_LEDS();
         lcd.print("Closing - twelve ");
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
