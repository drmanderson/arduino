//Pin connected to ST_CP of 74HC595-RELAY
int latchPinA = 8;
//Pin connected to SH_CP of 74HC595-RELAY
int clockPinA = 12;
//Pin connected to DS of 74HC595-RELAY
int dataPinA = 11;
//Pin connected to ST_CP of 74HC595-LED
int latchPinB = 3;
//Pin connected to SH_CP of 74HC595-LED
int clockPinB = 4;
//Pin connected to DS of 74HC595-LED
int dataPinB = 5;

const int list [] = {1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,32768};
#include <LiquidCrystal_I2C.h> // Library for LCD
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20 column and 4 rows


int datArray[16] = {0b0000000000000001,
                   0b0000000000000010,
                   0b0000000000000100,
                   0b0000000000001000,
                   0b0000000000010000,
                   0b0000000000100000,
                   0b0000000001000000,
                   0b0000000010000000,
                   0b0000000100000000,
                   0b0000001000000000,
                   0b0000010000000000,
                   0b0000100000000000,
                   0b0001000000000000,
                   0b0010000000000000,
                   0b0100000000000000,
                   0b1000000000000000
                  };

void setup() {
  //set pins to output because they are addressed in the main loop
  pinMode(latchPinA, OUTPUT);
  pinMode(clockPinA, OUTPUT);
  pinMode(dataPinA, OUTPUT);
  pinMode(latchPinB, OUTPUT);
  pinMode(clockPinB, OUTPUT);
  pinMode(dataPinB, OUTPUT);  
  lcd.init(); // initialize the lcd
  lcd.backlight();
}

void loop1() {
  //count up routine
  for (uint16_t element : list ) {
    uint16_t j=0;
    lcd.setCursor(0, 0);
    // // ground latchPin and hold low for as long as you are transmitting
    // lcd.print("setting latch low");
    // digitalWrite(latchPinA, LOW);
    // digitalWrite(latchPinB, LOW);
    // lcd.setCursor(0,1);
    // lcd.print("shifting out ");
    // lcd.print(j);
    // lcd.print ("     ");
    // shiftOut(dataPinA, clockPinA, LSBFIRST, j);
    // shiftOut(dataPinB, clockPinB, LSBFIRST, j);
    // shiftOut(dataPinB, clockPinB, LSBFIRST, (j>>8));
    // //return the latch pin high to signal chip that it
    // //no longer needs to listen for information
    // lcd.setCursor(0,2);
    // lcd.print("setting latch high");
    // digitalWrite(latchPinA, HIGH);
    // digitalWrite(latchPinB, HIGH);
    // delay(500);
    // ground latchPin and hold low for as long as you are transmitting
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("setting latch low");     
     j=element;
    digitalWrite(latchPinA, LOW);
    digitalWrite(latchPinB, LOW);
    lcd.setCursor(0,1);
    lcd.print("shifting out ");
    lcd.print(element);  
    lcd.print ("     ");
    shiftOut(dataPinA, clockPinA, LSBFIRST, j);
    shiftOut(dataPinB, clockPinB, LSBFIRST, j);
    shiftOut(dataPinB, clockPinB, LSBFIRST, (j >>8 ));
    //return the latch pin high to signal chip that it
    //no longer needs to listen for information
    digitalWrite(latchPinA, HIGH);
    digitalWrite(latchPinB, HIGH);
    delay(500);  
  } 
}void loop()
{
  // Count from 0 to 15
  for (int num = 0; num < 16; num++)
  {
    // ST_CP LOW to keep LEDs from changing while reading serial data
    digitalWrite(latchPinB, LOW);

    // Shift out the bits
    shiftOut(dataPinB, clockPinB, LSBFIRST, datArray[num]);
   shiftOut(dataPinB, clockPinB, LSBFIRST, (datArray[num]>> 8));
    // ST_CP HIGH change LEDs
    digitalWrite(latchPinB, HIGH);

    delay(500);
  }
   for (int num = 16; num > 0; num--)
  {
    // ST_CP LOW to keep LEDs from changing while reading serial data
    digitalWrite(latchPinB, LOW);

    // Shift out the bits
    shiftOut(dataPinB, clockPinB, LSBFIRST, datArray[num]);
   shiftOut(dataPinB, clockPinB, LSBFIRST, (datArray[num]>> 8));
    // ST_CP HIGH change LEDs
    digitalWrite(latchPinB, HIGH);

    delay(500);
  } 
}
