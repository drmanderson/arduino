
// 74HC595 LED PinOut
const int latchPinOut = 5;     // Connects to the RCLK pin of the 595 (12)  ST_CP
const int clockPinOut = 6;     // Connects to the SRCLK ping of the 595 (11) SH_CP
const int dataPinOut = 7                                                                                                                                                                                                                                                                                                                                                                                                                        ;      // Connects to the SER pin of the 595 (14) DS
const int OEPinOut = 4;        // Connects to OE pin of the 595 (13)
// 74HC595 RELAY pins
const int latchPinS = 16;     // Connects to the RCLK pin of the 595 (12)  ST_CP
const int clockPinS = 17;     // Connects to the SRCLK ping of the 595 (11) SH_CP
const int dataPinS = 15;      // Connects to the SER pin of the 595 (14) DS
const int OEPinS = 14;        // Connects to OE pin of the 595 (13)
const int readyLED = 2;

long LEDPattern = 0;

long RELAYPattern = 0;

void set_RELAYS()
{
  digitalWrite(OEPinS, LOW);
  digitalWrite(latchPinS, LOW);
  shiftOut(dataPinS, clockPinS, LSBFIRST, RELAYPattern);
  shiftOut(dataPinS, clockPinS, LSBFIRST, (RELAYPattern >> 8));
   digitalWrite(latchPinS, HIGH);
}

void set_LEDS()
{
  digitalWrite(OEPinOut, LOW);
  digitalWrite(latchPinOut, LOW);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, LEDPattern);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, (LEDPattern >> 8));
  digitalWrite(latchPinOut, HIGH);
}

void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("LED-pcb-test....");
  Serial.println("Setting up ....");

  // Ready LED PIN
  pinMode(readyLED, OUTPUT);

  // Setup 74HC595 LED serial connections
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);
  digitalWrite(OEPinOut, HIGH);
  pinMode(OEPinOut, OUTPUT);
  // Setup 74HC595 RELAY serial connections
  pinMode(latchPinS, OUTPUT);
  pinMode(clockPinS, OUTPUT);
  pinMode(dataPinS, OUTPUT);
  digitalWrite(OEPinS, HIGH);
  pinMode(OEPinS, OUTPUT);

}

void loop()
{

  // Light readyLED
  digitalWrite(readyLED, HIGH);
  LEDPattern = 0b1111111111111111;
  RELAYPattern = 0b1111111111111111;
  set_RELAYS();  
  set_LEDS();
  Serial.println("On");
  delay (1000);
  digitalWrite(readyLED, LOW);
  LEDPattern = 0b0000000000000000;
  RELAYPattern = 0b0000000000000000;
  set_RELAYS();
  set_LEDS();
  Serial.println("off");
  delay (1000);


}
