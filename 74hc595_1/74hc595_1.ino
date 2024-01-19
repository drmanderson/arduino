 /*
  74HC595 Shift Register with 7-segment LED display
  74hc595-7segdisplay.ino
  Count in hex from 0-F and display on 7-segment Common Cathode LED display

  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/

// Define Connections to 74HC595

// ST_CP pin 12
const int latchPin = 8;
// SH_CP pin 11
const int clockPin = 9;
// DS pin 14
const int dataPin = 10;

// Patterns for characters 0,1,2,3,4,5,6,7,8,9,A,b,C,d,E,F
int datArray[8] = {B00000001,
                   B00000010,
                   B00000100,
                   B00001000,
                   B00010000,
                   B00100000,
                   B01000000,
                   B10000000
                  };


void setup ()
{
  // Setup pins as Outputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  Serial.begin(9600);    
  Serial.println("74HC595 Demo 2xchips for 16 bit register.");
}
void loop()
{
  // Count from 0 to 15
  for (int num = 0; num < 16; num++)
  {
      Serial.print(num);
      Serial.print(", ");
      Serial.print(datArray[num],BIN);
      Serial.print(", 0x");
      Serial.print(datArray[num],HEX);
      Serial.print(", ");
      Serial.println(datArray[num]);
    // ST_CP LOW to keep LEDs from changing while reading serial data
    digitalWrite(latchPin, LOW);

    // Shift out the bits
    shiftOut(dataPin, clockPin, LSBFIRST, datArray[num]);

    // ST_CP HIGH change LEDs
    digitalWrite(latchPin, HIGH);

    delay(500);
  }
}
