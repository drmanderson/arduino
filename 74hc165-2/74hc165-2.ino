// HARDWARE CONNECTIONS
// Connect the following pins between your Arduino and the 74HC165 Breakout Board
// Connect pins A-H to 5V or GND or switches or whatever
const int dataIn = 5; // Connect Pin 5 to SER_OUT 7
const int load = 7; // Connect Pin 7 to SH/!LD 1
const int clockPin = 6; // Connect Pin 6 to CLK 2
const int clockEnablePin = 4; // Connect Pin 4 to !CE 15
byte incoming; // Variable to store the 8 values loaded from the shift register

// The part that runs once
void setup() 
{                
  // Initialize serial to gain the power to obtain relevant information, 9600 baud
  Serial.begin(9600);

  // Initialize each digital pin to either output or input
  // We are commanding the shift register with each pin with the exception of the serial
  // data we get back on the data_pin line.
  pinMode(load, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataIn, INPUT);

//  // Required initial states of these two pins according to the datasheet timing diagram
//  digitalWrite(clk_pin, HIGH);
//  digitalWrite(shld_pin, HIGH);

}

// The part that runs to infinity and beyond
void loop() {


  // Write pulse to load pin
  digitalWrite(load, LOW);
  delayMicroseconds(10);
  digitalWrite(load, HIGH);
  delayMicroseconds(10);
 
  // Get data from 74HC165
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockEnablePin, LOW);
  byte incoming = shiftIn(dataIn, clockPin, LSBFIRST);
  digitalWrite(clockEnablePin, HIGH);
 
  // Print to serial monitor
  Serial.print("Pin States:   ");
  Serial.println(incoming, BIN);
  delay(500);

}
