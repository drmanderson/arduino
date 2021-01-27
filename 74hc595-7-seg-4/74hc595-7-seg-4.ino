#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 5, 4, 3, 2);
#define latchPin  12  // Pin connected to ST_CP of 74HC595, Pin12
#define clockPin  13  // Pin connected to SH_CP of 74HC595, Pin11
#define dataPin  11   // Pin connected to DS of 74HC595, Pin14

// Pins driving common cathodes
#define CA1 9 // segmen1
#define CA2 10 // segment2

// digit pattern for a 7-segment display
const byte digit_pattern[16] =
{
  B00111111,  // 0
  B00000110,  // 1
  B01011011,  // 2
  B01001111,  // 3
  B01100110,  // 4
  B01101101,  // 5
  B01111101,  // 6
  B00000111,  // 7
  B01111111,  // 8
  B01101111,  // 9
  B01110111,  // A
  B01111100,  // b
  B00111001,  // C
  B01011110,  // d
  B01111001,  // E
  B01110001   // F
};

void setup() {
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode (CA1, OUTPUT);
  pinMode (CA2, OUTPUT);
    // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Hello, Sam");
  lcd.setCursor(0, 1);
  lcd.print("Number: ");
}



void lightDigit1(int number) {
  digitalWrite(CA1, LOW);
  digitalWrite(CA2, HIGH);
  update_one_digit(number);
}

void lightDigit2(int number) {
  digitalWrite(CA1, HIGH);
  digitalWrite(CA2, LOW);
  update_one_digit(number);
}

void update_one_digit(int data) {
  byte pattern;
  // get the digit pattern to be updated
  pattern = digit_pattern[data];
  // turn off the output of 74HC595
  digitalWrite(latchPin, LOW);
  // update data pattern to be outputed from 74HC595
  shiftOut(dataPin, clockPin, MSBFIRST, pattern);
  // turn on the output of 74HC595
  digitalWrite(latchPin, HIGH);
}

void lcd_display (int digit1, int digit2) {

  String stringOne = String (digit1, HEX);
  String stringTwo = String (digit2, HEX);
  stringOne.toUpperCase();
  stringTwo.toUpperCase();
  String stringOut = stringOne+stringTwo;
  lcd.setCursor(8,2);
  lcd.print( stringOut );

}

void loop() {
  for (int digit1=0; digit1 < 16; digit1++) {
    for (int digit2=0; digit2 < 16; digit2++) {
      lcd_display (digit1, digit2);
      unsigned long startTime = millis();
      for (unsigned long elapsed=0; elapsed < 600; elapsed = millis() - startTime) {
        lightDigit1(digit1);
        delay(10);
        lightDigit2(digit2);
        delay(10);
      }
    }
  }
}
