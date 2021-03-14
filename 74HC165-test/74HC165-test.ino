#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4); 

// include the PWMServo library
#include <Adafruit_PWMServoDriver.h>

// Setup pwm objects and their addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);

// HARDWARE CONNECTIONS
// Connect the following pins between your Arduino and the 74HC165 Breakout Board
// Connect pins A-H to 5V or GND or switches or whatever
const int clockEnablePin  = 4;  // Connects to Clock Enable pin the 165 (15)
const int dataPin         = 5;  // Connects to the Q7 pin the 165 (9)
const int clockPin        = 6;  // Connects to the Clock pin the 165 (2)
const int ploadPin        = 7;  // Connects to Parallel load pin the 165 (1)

#define BYTES_VAL_T unsigned long 
#define NUMBER_OF_CHIPS 3
#define DATA_WIDTH NUMBER_OF_CHIPS * 8
#define NUMSERVOS 5

struct ServoData {
  Adafruit_PWMServoDriver  pwmcard;        // PCA9685 card number
  int  pwmcard_socket; // Socket on PCA9685 card
  int  openangle;      // User Configurable servo angle for open point
  int  closeangle;     // User Configurable servo angle for close point
};
ServoData servo[NUMSERVOS];


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
    bitVal = digitalRead(dataPin);
    /* Set the corresponding bit in bytesVal
     */
     
     bytesVal |= (bitVal << (DATA_WIDTH-1 - i));

    /* Pulse the Clock - rising edge shifts next bit in
     */
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
  }
  return(bytesVal);
}

void move_points(int switchNum) {
  switch(switchNum){
    case 1:
      // Button "1:" - close juction one
      Serial.println("Button one triggered");
      break; 

    case 2:
      // Button "2:" - open juction one
      Serial.println("Button two triggered");
      break;              

    case 3:
      // Button "3:" - close juction two
      Serial.println("Button three triggered");
      break;   
  
    case 4:
      // Button "4:" - open juction two
      Serial.println("Button four triggered");
      break; 
  
    case 5:
      // Button "5:" - close juction one
      Serial.println("Button five triggered");
      break; 

    case 6:
      // Button "6:" - close juction one
      Serial.println("Button six triggered");
      break;              

    case 7:
      // Button "7:" - close juction one
      Serial.println("Button seven triggered");
      break;   
  
    case 8:
      // Button "8:" - close juction one
      Serial.println("Button eight triggered");
      break;

    case 9:
      // Button "1:" - close juction one
      Serial.println("Button nine triggered");
      break; 

    case 10:
      // Button "2:" - open juction one
      Serial.println("Button ten triggered");
      break;              

    case 11:
      // Button "3:" - close juction two
      Serial.println("Button eleven triggered");
      break;   
  
    case 12:
      // Button "4:" - open juction two
      Serial.println("Button twelve triggered");
      break; 
  
    case 13:
      // Button "5:" - close juction one
      Serial.println("Button thirteen triggered");
      break; 

    case 14:
      // Button "6:" - close juction one
      Serial.println("Button fourteen triggered");
      break;              

    case 15:
      // Button "7:" - close juction one
      Serial.println("Button fifteen triggered");
      break;   
  
    case 16:
      // Button "8:" - close juction one
      Serial.println("Button sixteen triggered");
      break;  

    case 17:
      // Button "8:" - close juction one
      Serial.println("Button seventeen triggered");
      break;

    case 18:
      // Button "1:" - close juction one
      Serial.println("Button eighteen triggered");
      break; 

    case 19:
      // Button "2:" - open juction one
      Serial.println("Button nineteen triggered");
      break;              

    case 20:
      // Button "3:" - close juction two
      Serial.println("Button twenty triggered");
      break;   
  
    case 21:
      // Button "4:" - open juction two
      Serial.println("Button twentyone triggered");
      break; 
  
    case 22:
      // Button "5:" - close juction one
      Serial.println("Button twentytwo triggered");
      break; 

    case 23:
      // Button "6:" - close juction one
      Serial.println("Button twentythree triggered");
      break;              

    case 24:
      // Button "7:" - close juction one
      Serial.println("Button twentyfour triggered");
      break;   
  
    default:
      // Button "8:" - close juction one
      Serial.println("default");
      break;       
  }
}

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
  // Required initial states of these two pins according to the datasheet timing diagram
  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin,HIGH);
}

void loop(){
  // Read all pin values
  pinValues = read_values();
  // Process any changes in the values and call to move_points
  if (pinValues != oldPinValues) {
    for (int i=0;i< DATA_WIDTH; i++){
      if ((pinValues >> i) & 1) {
        Serial.println("i");
        move_points(i);
      }
   }
    oldPinValues = pinValues;
  }
  delay(200);
}
