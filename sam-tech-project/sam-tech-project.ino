
// include IR Library:
#include <IRremote.h>

// include LCD library:
#include <LiquidCrystal.h>

// Define Pins
const int IRpin=11;
const int BLUE=6;
const int GREEN=7;
const int RED=8;

// initialize the LCD library with the numbers of the interface pins
LiquidCrystal lcd(10 , 9, 5, 4, 3, 2);

// initialiae the IR library and start it
IRrecv irrecv(IRpin);
decode_results results;

void setup()
{
  // Setup IR receiver on pin 11
  irrecv.enableIRIn();
  // Setup the RGB LED Pins
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // Show a quick startup sequence
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Tech Project");
  lcd.setCursor(0,1);
  lcd.print("Start Up...."); 
  delay(2000); 
  lcd.setCursor(0,1);
  lcd.print("                 ");
  sequence ();
  all_off();
  lcd.setCursor(0,1);
  lcd.print("                 ");
}

// define variables
int redValue;
int greenValue;
int blueValue;

// main loop
void loop()
{

    if (irrecv.decode(&results)) { 
      
    switch (results.value) {
      case 0x9716BE3F:
        red_led_on();
        break;

      case 0x3D9AE3F7:
        green_led_on();  
        break;

      case 0x6182021B:
        blue_led_on();
        break;

      case 0x3EC3FC1B:
        sequence();
        break;
       
       case 0xC101E57B:
         all_off();   
         break;  

       
    }
    irrecv.resume();   // Receive the next value
  }
}


void sequence () {
  red_led_on();
  delay(1000);
  green_led_on();  
  delay(1000);
  blue_led_on();
  delay(1000); 
}

void all_off() {
  redValue = LOW;
  greenValue = LOW;
  blueValue = LOW;
  digitalWrite(RED, redValue);
  digitalWrite(GREEN, greenValue);
  digitalWrite(BLUE, blueValue);
  lcd.setCursor(4,1);
  lcd.print("All off      ");  
}

void red_led_on () {
  redValue = HIGH;
  greenValue = LOW;
  blueValue = LOW;
  digitalWrite(RED, redValue);
  digitalWrite(GREEN, greenValue);
  digitalWrite(BLUE, blueValue);
  lcd.setCursor(4,1);
  lcd.print("Red LED    ");
}

void green_led_on () {
  redValue = LOW;
  greenValue = HIGH;
  blueValue = LOW;
  digitalWrite(RED, redValue);
  digitalWrite(GREEN, greenValue);
  digitalWrite(BLUE, blueValue);
  lcd.setCursor(4,1);
  lcd.print("Green LED     ");
}

void blue_led_on () {
  redValue = LOW;
  greenValue = LOW;
  blueValue = HIGH;
  digitalWrite(RED, redValue);
  digitalWrite(GREEN, greenValue);
  digitalWrite(BLUE, blueValue); 
  lcd.setCursor(4,1);
  lcd.print("Blue LED     "); 
}
