#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// include the PWMServo library
#include <Adafruit_PWMServoDriver.h>
// Setup pwm objects and their addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);

// Lets put our defines here.
#define NUMSERVOS 28 // Enter the number of servos being controlled
#define SERVOMIN 125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 575 // this is the 'maximum' pulse length count (out of 4096)
#define BYTES_VAL_T unsigned long
#define NUMBER_OF_CHIPS 4
#define DATA_WIDTH NUMBER_OF_CHIPS * 8

// Setup servo struct that holds pwm card, socket number and open/close angles.
struct ServoData
{
  Adafruit_PWMServoDriver pwmcard; // PCA9685 card number
  byte pwmcard_socket;             // Socket on PCA9685 card
  byte openangle;                  // User Configurable servo angle for open point
  byte closeangle;                 // User Configurable servo angle for close point
  int relay = 0;                   // Is a relay needed for FROG switching
  byte currangle = 90;                  // Current angle - used for slow sweep
};

ServoData servo[NUMSERVOS] ;

// 74HC165 pins
const int clockEnablePin = 4; // Connects to Clock Enable pin the 165 (15)
const int dataPin = 5;        // Connects to the Q7 pin the 165 (9)
const int clockPin = 6;       // Connects to the Clock pin the 165 (2)
const int ploadPin = 7;       // Connects to Parallel load pin the 165 (1)

// 74HC595 LED pins
const int latchPinOut = 9;  // Connects to the RCLK pin of the 595 (12)  ST_CP
const int clockPinOut = 10; // Connects to the SRCLK ping of the 595 (11) SH_CP
const int dataPinOut = 11;  // Connects to the SER pin of the 595 (14) DS

// 74HC595 RELAY pins
const int latchPinS = 14; // Connects to the RCLK pin of the 595 (12)  ST_CP
const int clockPinS = 15; // Connects to the SRCLK ping of the 595 (11) SH_CP
const int dataPinS = 16;  // Connects to the SER pin of the 595 (14) DS
const int OEPinS = 17;    // Connects to OE pin of the 595 (13)

const int readyLED = 2;

uint16_t LEDpattern1; // LED Pattern to send to the 74HC595 chips
uint16_t tempVal;
uint16_t LEDArray[16] = {0b0000000000000001,
                         0b0000000000000010,
                         0b0000000000000100,
                         0b0000000000001000,
                         0b0000000000010000,
                         0b0000000000100000,
                         0b0000000001000000,
                         0b0000000100000000,
                         0b0000001000000000,
                         0b0000010000000000,
                         0b0000100000000000,
                         0b0001000000000000,
                         0b0010000000000000,
                         0b0100000000000000,
                         0b1000000000000000};

uint8_t RELAYPattern = 0;
uint8_t relayArray[8] = {1, 2, 4, 8, 16, 32, 64, 128};
// Setup initial values
BYTES_VAL_T pinValues = 0;    // new values from  74HC165 chips
BYTES_VAL_T oldPinValues = 0; // old values from  74HC165 chips

BYTES_VAL_T read_values()
{

  BYTES_VAL_T bitVal = 0;
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
  for (int i = 0; i < DATA_WIDTH; i++)
  {
    // Read value in
    bitVal = digitalRead(dataPin);

    // Set the corresponding bit in bytesVal
    bytesVal |= (bitVal << (DATA_WIDTH - 1 - i));

    // Pulse the Clock - rising edge shifts next bit in
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
  }
  return (bytesVal);
}

void move_servo(int srv, bool open, bool off, bool init = false)
{
  byte ang = 90; // 90 is a safe agnle to set to if something is borked
  byte pulselength = 0;
  // Are we initialising - init=true
  if (init)
  {
    Serial.println("init");
    // currangle is not valid. Close  points and set currangle to closeangle
    pulselength = map(servo[srv].closeangle, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
    servo[srv].pwmcard.setPWM(servo[srv].pwmcard_socket, 0, pulselength);
    if (off)
    { // There's no real load on the servo so we can disable it after it's moved. Saves a bit of power and stops it hunting
      servo[srv].pwmcard.setPWM(srv, 0, 4096);
    }
    servo[srv].currangle = servo[srv].closeangle;
    Serial.println(servo[srv].currangle);
  }
  else // OK - not initializing - system is up and running - we're opening or closing a point
  {
    if (open)
    { // We want to open the point
      ang = servo[srv].openangle;
      // If there is a servo associated with the point set it to be activated.
      // Definition - open is NC on the Servo
      //              closed is NO on the Servo
      if (servo[srv].relay > 0 ) {
        // Bitlogin to control relays as using 74HC595 chip's outputs to hold the servo open
        RELAYPattern = RELAYPattern | (relayArray[servo[srv].relay] - 1);
      }
    }
    else
    { // We want to close the point
      ang = servo[srv].closeangle;
      if (swervo[srv].relay > 0 ){
        // If there is a relay associated with the point closed if NO so switch off the 74HC595 pin
        tempVal = RELAYPattern;
        RELAYPattern = bitclear(tempVal, (servo[srv].relay - 1))
      }
    }
    if (servo[srv].currangle == ang)
    {
      // Serial.println("currangle is set already");
      return;
    }
    else if (ang > servo[srv].currangle)
    {
      int diff = ang - servo[srv].currangle;
      for (int i = servo[srv].currangle; i < ang; i++)
      {
        int pulselength = map(i, 0, 180, SERVOMIN, SERVOMAX);
        // Serial.println(pulselength);
        servo[srv].pwmcard.setPWM(servo[srv].pwmcard_socket, 0, pulselength);
        if (diff >= i ){
          set_relay();
        }
        delay(30);
      }
      servo[srv].currangle = ang;
    }
    else if (servo[srv].currangle > ang)
    {
      int diff = servo[srv].currangle - ang;
      for (int i = servo[srv].currangle; i > ang; i--)
      {
        int pulselength = map(i, 0, 180, SERVOMIN, SERVOMAX);
        servo[srv].pwmcard.setPWM(servo[srv].pwmcard_socket, 0, pulselength);
        if {i <= diff} {
          set_relay();
        }
        // Serial.println(pulselength);
        delay(30);
      }
      servo[srv].currangle = ang;
    }
  }
}

void setup()
{
  // Setup i2c LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setting up ....");
  delay(2000);
  lcd.clear();
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("railway-servo-LED....");
  Serial.println("Setting up ....");

  // Setup 74HC165 SWITCH Serial connections
  pinMode(ploadPin, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  // Required initial states of these two pins according to the datasheet timing diagram
  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin, HIGH);

  // Setup 74HC595 LED serial connections
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);

  // Ready LED PIN
  pinMode(readyLED, OUTPUT);

  // Setup 74HC595 RELAY serial connections
  pinMode(latchPinS, OUTPUT);
  pinMode(clockPinS, OUTPUT);
  pinMode(dataPinS, OUTPUT);
  pinMode(OEPinS, OUTPUT);
  digitalWrite(OEPinS, LOW);

  // initialise the 74HC595 with all LED off 0b0000000000000000
  LEDpattern1 = 0b0000000000000000;
  set_LEDS();

  // Setup PWM PCA9685 cards
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  pwm1.begin();
  pwm1.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  pwm2.begin();
  pwm2.setPWMFreq(60); // Analog servos run at ~60 Hz updates

  // Setup NUMSERVOS servos
  // Remember to update ths number when new servos are added.
  servo[0].pwmcard = pwm;
  servo[0].pwmcard_socket = 0;
  servo[0].openangle = 145;
  servo[0].closeangle = 65;
  servo[0].currangle = servo[0].openangle;

  servo[1].pwmcard = pwm;
  servo[1].pwmcard_socket = 1;
  servo[1].openangle = 110;
  servo[1].closeangle = 60;
  servo[1].currangle = servo[1].openangle;

  servo[2].pwmcard = pwm;
  servo[2].pwmcard_socket = 2;
  servo[2].openangle = 120;
  servo[2].closeangle = 60;
  servo[2].currangle = servo[2].openangle;

  servo[3].pwmcard = pwm;
  servo[3].pwmcard_socket = 3;
  servo[3].openangle = 125;
  servo[3].closeangle = 55;
  servo[3].currangle = servo[3].openangle;

  servo[16].pwmcard = pwm1;
  servo[16].pwmcard_socket = 0;
  servo[16].openangle = 130;
  servo[16].closeangle = 60;
  servo[16].currangle = servo[16].openangle;

  servo[17].pwmcard = pwm1;
  servo[17].pwmcard_socket = 1;
  servo[17].openangle = 70;
  servo[17].closeangle = 130;
  servo[17].currangle = servo[17].openangle;

  servo[18].pwmcard = pwm1;
  servo[18].pwmcard_socket = 2;
  servo[18].openangle = 60;
  servo[18].closeangle = 120;
  servo[18].currangle = servo[18].openangle;

  servo[19].pwmcard = pwm1;
  servo[19].pwmcard_socket = 3;
  servo[19].openangle = 135;
  servo[19].closeangle = 60;
  servo[19].currangle = servo[19].openangle;

  servo[20].pwmcard = pwm1;
  servo[20].pwmcard_socket = 4;
  servo[20].openangle = 60;
  servo[20].closeangle = 120;
  servo[20].currangle = servo[20].openangle;

  servo[21].pwmcard = pwm1;
  servo[21].pwmcard_socket = 5;
  servo[21].openangle = 120;
  servo[21].closeangle = 60;
  servo[21].currangle = servo[21].openangle;

  servo[22].pwmcard = pwm1;
  servo[22].pwmcard_socket = 6;
  servo[22].openangle = 65;
  servo[22].closeangle = 130;
  servo[22].currangle = servo[22].openangle;

  servo[23].pwmcard = pwm1;
  servo[23].pwmcard_socket = 7;
  servo[23].openangle = 65;
  servo[23].closeangle = 135;
  servo[23].currangle = servo[23].openangle;

  servo[24].pwmcard = pwm1;
  servo[24].pwmcard_socket = 8;
  servo[24].openangle = 130;
  servo[24].closeangle = 60;
  servo[24].currangle = servo[24].openangle;

  servo[25].pwmcard = pwm1;
  servo[25].pwmcard_socket = 9;
  servo[25].openangle = 120;
  servo[25].closeangle = 70;
  servo[25].currangle = servo[25].openangle;

  servo[26].pwmcard = pwm1;
  servo[26].pwmcard_socket = 10;
  servo[26].openangle = 140;
  servo[26].closeangle = 60;
  servo[26].currangle = servo[26].openangle;

  servo[27].pwmcard = pwm1;
  servo[27].pwmcard_socket = 11;
  servo[27].openangle = 120;
  servo[27].closeangle = 55;
  servo[27].currangle = servo[27].openangle;

  // Start board in safe positions with all junctions "closed"
  for (int i = 0; i <= NUMSERVOS - 1; i++)
  {
    if (servo[i].openangle != 0)
    { // We want to ignore elements not yet filled on the PWM9685 card
      Serial.print("Closing :");
      Serial.println(i);
      // move_servo(servo[i].pwmcard_socket, false, true, true);
      move_servo(i, false, true, true);
      delay(100);
    }
  }
  Serial.println("Flashing Leds");
  // Flash all the LEDS
  LEDpattern1 = 0b1111111111111111;
  set_LEDS();
  delay(500);
  LEDpattern1 = 0b0000000000000000;
  set_LEDS();
  delay(500);
  LEDpattern1 = 0b1111111111111111;
  set_LEDS();
  delay(500);
  LEDpattern1 = 0b0000000000000000;
  set_LEDS();
  delay(500);
  lcd.print("Setup done.");
  Serial.println("Setup done.");
}

void set_LEDS()
{
  // Set the LEDS that are on / off
  digitalWrite(latchPinOut, LOW);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, LEDpattern1);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, (LEDpattern1 >> 8));
  digitalWrite(latchPinOut, HIGH);
}

void set_relay(uint16_t)
{
  Serial.print("RELAYPattern... ");
  Serial.println(RELAYPattern);
  // ST_CP LOW to keep LEDs from changing while reading serial data
  digitalWrite(latchPinS, LOW);
  // Shift out the bits
  shiftOut(dataPinS, clockPinS, LSBFIRST, RELAYPattern);
  // ST_CP HIGH change LEDs
  digitalWrite(latchPinS, HIGH);
}

void print_message(String message, bool lcdout = true, bool serialout = true)
{
  if (serialout)
  {
    Serial.println(message);
  }
  if (lcdout)
  {
    lcd.clear();
    lcd.home();
    lcd.print(message);
  }
}

void move_points(int switchNum)
{
  // Move either one or two servos based on which button has been pressed
  switch (switchNum)
  {

  case 0:
    // Button "1:" - Close juction A
    print_message("Button 1.", false, true);
    print_message("Closing - A");
    move_servo(16, false, false, false);
    // move_servo(servo[16].pwmcard, servo[16].pwmcard_socket, servo[16].closeangle );
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 0);
    set_LEDS();
    delay(500);
    break;

  case 1:
    // Button "2" - Open juntion A
    print_message("Button 2.", false, true);
    print_message("Opening - A");
    move_servo(16, true, false, false); // open point and turn off servo
    // move_servo(servo[16].pwmcard, servo[16].pwmcard_socket, servo[16].openangle );
    tempVal = LEDpattern1;
    LEDpattern1 = LEDpattern1 | LEDArray[0];
    set_LEDS();
    delay(500);
    break;

  case 2:
    // Button "3" - Close Juction B
    print_message("Button 3.", false, true);
    print_message("Closing - B");
    move_servo(17, false, false, false); // close point and turn off servo
                                         // move_servo(servo[17].pwmcard, servo[17].pwmcard_socket, servo[17].closeangle );
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 1);
    set_LEDS();
    delay(500);
    break;

  case 3:
    // Button "4" - Open Juction B
    print_message("Button 4.", false, true);
    print_message("Opening - B");
    move_servo(17, true, false, false); // open point and turn off servo
    // move_servo(servo[17].pwmcard, servo[17].pwmcard_socket, servo[17].openangle );
    LEDpattern1 = LEDpattern1 | LEDArray[1];
    set_LEDS();
    delay(500);
    break;

  case 4:
    // Button "5" - Close Juction C
    print_message("Button 5.", false, true);
    print_message("Closing - C ");
    move_servo(18, false, true); // Close point and turn off servo
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 2);
    set_LEDS();
    delay(500);
    break;

  case 5:
    // Button "6" - Open Juction C
    print_message("Button 6.", false, true);
    print_message("Opening - C");
    move_servo(18, true, true); // open point and turn off servo
    LEDpattern1 = LEDpattern1 | LEDArray[2];
    set_LEDS();
    delay(500);
    break;

  case 6:
    // Button "7" - Close Juction D
    print_message("Button 7.", false, true);
    print_message("Closing - D ");
    move_servo(2, false, true);  // close point and turn off servo
    move_servo(19, false, true); // close point and turn off servo
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 3);
    set_LEDS();
    delay(500);
    break;

  case 7:
    // Button "8" - Open Juction D
    print_message("Button 8.", false, true);
    print_message("Opening - D");
    move_servo(2, true, true);  // open point and turn off servo
    move_servo(19, true, true); // open point and turn off servo
    LEDpattern1 = LEDpattern1 | LEDArray[3];
    set_LEDS();
    delay(500);
    break;

  case 8:
    // Button "9" - Close Juction E
    print_message("Button 9.", false, true);
    print_message("Closing - E");
    move_servo(20, false, true); // Close point and turn off servo
    move_servo(21, false, true); // Close point and turn off servo
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 4);
    set_LEDS();
    delay(500);
    break;

  case 9:
    // Button "10" - Open Juction E
    print_message("Button 10.", false, true);
    move_servo(20, true, true); // Open point and turn off servo
    move_servo(21, true, true); // Open point and turn off servo
    LEDpattern1 = LEDpattern1 | LEDArray[4];
    set_LEDS();
    print_message("Opening - E ");
    delay(500);
    break;

  case 10:
    // Button "11" - Close Juction F
    print_message("Button 11.", false, true);
    print_message("Closing - F ");
    move_servo(22, false, true);
    move_servo(23, false, true);
    LEDpattern1 = LEDpattern1 | LEDArray[5];
    set_LEDS();
    delay(500);
    break;

  case 11:
    // Button "12" - Open Juction F
    print_message("Button 12.", false, true);
    print_message("Opening - F ");
    move_servo(22, true, true);
    move_servo(23, true, true);
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 5);
    set_LEDS();
    delay(500);
    break;

  case 12:
    // Button "13" - Close Juction G
    print_message("Button 13.", false, true);
    print_message("Closing - G ");
    move_servo(24, false, true);
    LEDpattern1 = LEDpattern1 | LEDArray[6];
    set_LEDS();
    delay(500);
    break;

  case 13:
    // Button "14" - Open Juction G
    print_message("Button 14.", false, true);
    print_message("Openging - G ");
    move_servo(24, true, true);
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 6);
    set_LEDS();
    delay(500);
    break;

  case 14:
    // Button "15" - CLose Juction H
    print_message("Button 15.", false, true);
    print_message("Closing - H");
    move_servo(25, false, true);
    LEDpattern1 = LEDpattern1 | LEDArray[7];
    set_LEDS();
    delay(500);
    break;

  case 15:
    // Button "16" - Open Juction H
    print_message("Button 16.", false, true);
    print_message("Opening - H ");
    move_servo(25, true, true);
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 7);
    set_LEDS();

    delay(500);
    break;

  case 16:
    // Button "17" - Close Juction I
    print_message("Button 17.", false, true);
    print_message("Closing - I");
    move_servo(26, false, true);
    LEDpattern1 = LEDpattern1 | LEDArray[8];
    set_LEDS();
    delay(500);
    break;

  case 17:
    // Button "18" - Open Juction I
    print_message("Button 18.", false, true);
    print_message("Opening - I ");
    move_servo(26, true, true);
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 8);
    set_LEDS();
    delay(500);
    break;

  case 18:
    // Button "19" - close Juction J
    print_message("Button 19.", false, true);
    print_message("Closing - J ");
    move_servo(27, false, true);
    LEDpattern1 = LEDpattern1 | LEDArray[9];
    set_LEDS();
    delay(500);
    break;

  case 19:
    // Button "20" - open Juction J
    print_message("Button 20.", false, true);
    print_message("Opening - J ");
    move_servo(27, true, true);
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 9);
    set_LEDS();
    delay(500);
    break;

  case 20:
    // Button "21" - close Juction k
    print_message("Button 21.", false, true);
    print_message("Closing - k");
    move_servo(0, false, true);
    move_servo(1, false, true);
    LEDpattern1 = LEDpattern1 | LEDArray[10];
    set_LEDS();
    delay(500);
    break;

  case 21:
    // Button "22" - open Juction k
    print_message("Button 22.", false, true);
    print_message("Opening - k ");
    move_servo(0, true, true);
    move_servo(1, true, true);
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 10);
    set_LEDS();
    delay(500);
    break;

  case 22:
    // Button "21" - Close Juction L
    print_message("Button 22.", false, true);
    print_message("Closing - L");
    move_servo(3, false, true);
    LEDpattern1 = LEDpattern1 | LEDArray[11];
    set_LEDS();

    delay(500);
    break;

  case 23:
    // Button "23" - Open Juction L
    print_message("Button 23.", false, true);
    print_message("Opening - L ");
    move_servo(3, true, true);
    tempVal = LEDpattern1;
    LEDpattern1 = bitClear(tempVal, 11);
    set_LEDS();
    delay(500);
    break;
  default:
    // default - it's broken
    String message;
    message = "Default: " + switchNum;
    print_message(message);
    print_message("Failure...");
    break;
  }
}

void loop()
{
  // Light readyLED
  digitalWrite(readyLED, HIGH);
  // Read all pin values
  pinValues = read_values();
  // Process any changes in the values and call to move_points
  if (pinValues != oldPinValues)
  {
    for (int i = 0; i < DATA_WIDTH; i++)
    {
      if ((pinValues >> i) & 1)
      {
        // We're doing something so turn off readyLED
        digitalWrite(readyLED, LOW);
        move_points(i);
      }
    }
    oldPinValues = pinValues;
    // Set LED to reflect new settings
    set_LEDS();
  }
  delay(200);
}
