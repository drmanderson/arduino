// HARDWARE CONNECTIONS
// Connect the following pins between your Arduino and the 74HC165 Breakout Board
// Connect pins A-H to 5V or GND or switches or whatever
const int clockEnablePin  = 4;  // Connects to Clock Enable pin the 165 (15)
const int dataPin         = 5;  // Connects to the Q7 pin the 165 (9)
const int clockPin        = 6;  // Connects to the Clock pin the 165 (2)
const int ploadPin        = 7;  // Connects to Parallel load pin the 165 (1)

byte incoming1;                 // new values for first 74HC165 chip


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
  digitalWrite(clockPin, HIGH);
  digitalWrite(ploadPin,HIGH);

}

byte read_values() {

  byte data = 0;
  // Write pulse to load pin
  digitalWrite(ploadPin, LOW);
  delayMicroseconds(5);
  digitalWrite(ploadPin, HIGH);
  delayMicroseconds(5);
 
  // Get data from 74HC165 chips
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockEnablePin, LOW);
  data = shiftIn(dataPin, clockPin, MSBFIRST);
  Serial.print("SR 1: ");
  Serial.println(data, BIN);

  digitalWrite(clockPin, HIGH);
  
  return data;

}

void loop()
{
  incoming1 = read_values();
  Serial.print("Data is: ");
  Serial.println(incoming1, BIN);

  delay(500);
}
