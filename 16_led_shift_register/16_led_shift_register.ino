int latchPinOut     = 9;  // ST_CP pin 12
int clockPinOut     = 10;  // SH_CP pin 11
int dataPinOut      = 11;  // DS pin 14
byte LEDpattern;        // LED Pattern to send to the 74hc595s
byte LEDpattern1;
byte LEDpattern2;


int LEDArray[9] = {B00000001, B00000010, B00000100, B00001000, B00010000, B00100000,B01000000,B10000000,B00000000};

void setup (){
  Serial.begin(9600);
  Serial.println("Starting");
  // Setup 74HC696 serial connections
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);

  LEDpattern = B11111111;
  Serial.println ("Setting LEDS to 11111111");
  set_LEDS (LEDpattern, LEDpattern);
  delay(1000);
  Serial.println ("Setting LEDS to 00000000");
  LEDpattern = B00000000;
  set_LEDS (LEDpattern,LEDpattern);  
  delay(1000);
  LEDpattern = B11111111;
}

void set_LEDS ( char LEDpattern1 , char LEDpattern2) {
  // Set the LEDS that are on / off 
  digitalWrite(latchPinOut, LOW);
  shiftOut (dataPinOut, clockPinOut, MSBFIRST, LEDpattern1 );
  shiftOut (dataPinOut, clockPinOut, MSBFIRST, LEDpattern2 );
  digitalWrite (latchPinOut, HIGH);
}

void loop () {
  set_LEDS( B11111111,B11111111);
  delay (1000);
  set_LEDS( B00000000,B10100000);   
  delay (1000);
  set_LEDS( B10100000,B10100000);   
  delay (1000);  
}
