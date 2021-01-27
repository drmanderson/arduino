#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 5, 4, 3, 2);
unsigned int digit_base;
int latchPin = 12; // Pin connected to ST_CP of 74HC595, Pin12 
int clockPin = 13; // Pin connected to SH_CP of 74HC595, Pin11 
int dataPin = 11; // Pin connected to DS of 74HC595, Pin14 
//display 0,1,2,3,4,5,6,7,8,9,A,b,C,d,E,F
//int datArray[16] = {252, 96, 218, 242, 102, 182, 190, 224, 254, 246, 238, 62, 156, 122, 158, 142};

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

unsigned int counter = 0;
void setup()
{
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);  
    // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Hello, Sam");
  lcd.setCursor(0, 1);
  lcd.print("Number: ");
}
 
void update_one_digit(int data)
{
  int i;
  byte pattern;
  
  // get the digit pattern to be updated
  pattern = digit_pattern[data];

  // turn off the output of 74HC595
  digitalWrite(latchPin, LOW);
  
  // update data pattern to be outputed from 74HC595
  // because it's a common anode LED, the pattern needs to be inverted
  shiftOut(dataPin, clockPin, MSBFIRST, pattern);
  lcd.setCursor(9, 1);
  String stringOne = String (counter % digit_base , HEX);
  stringOne.toUpperCase();
  lcd.print( stringOne );  
  // turn on the output of 74HC595
  digitalWrite(latchPin, HIGH);
}

void loop()
{ 
  int i;
  
  
  counter++;
  
  digit_base = 16;

  // get the value to be displayed and update one digit
  update_one_digit(counter % digit_base);
  
  delay(500);
}
