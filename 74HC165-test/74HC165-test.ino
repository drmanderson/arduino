// HARDWARE CONNECTIONS
// Connect the following pins between your Arduino and the 74HC165 Breakout Board
// Connect pins A-H to 5V or GND or switches or whatever
const int clockEnablePin  = 4;  // Connects to Clock Enable pin the 165 (15)
const int dataPin         = 5;  // Connects to the Q7 pin the 165 (9)
const int clockPin        = 6;  // Connects to the Clock pin the 165 (2)
const int ploadPin        = 7;  // Connects to Parallel load pin the 165 (1)

#define BYTES_VAL_T unsigned long 
#define NUMBER_OF_CHIPS 3
#define DATA_WIDTH NUMBER_OF_CHIPS * 8

BYTES_VAL_T pinValues;                    // new values for  74HC165 chips
BYTES_VAL_T oldPinValues;                 // old values for  74HC165 chips

BYTES_VAL_T read_values() {

  unsigned long bitVal;
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
  for (int i=0; i < DATA_WIDTH; i++) {
    bitVal = digitalRead(dataPin);
    /* Set the corresponding bit in bytesVal
     */
    bytesVal |= (bitVal << (DATA_WIDTH-1 - i));

    /* Pulse the Clock - rising edge shifts next bit in
     */
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
  }


  return(bytesVal);
}

void display_pin_values() {
  
  Serial.print("Pin states:\r\n");
  
  for (int i=0;i< DATA_WIDTH; i++){
    Serial.print(" Pin-");
    Serial.print(i);
    Serial.print(": ");

    if ((pinValues >> i) & 1)
      Serial.print("HIGH");
    else
      Serial.print("LOW");

    Serial.print("\r\n");
  }

  Serial.print("\r\n");
}



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
  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin,HIGH);

  /*
   * Read in and display pin states
   */

  pinValues = read_values();
  display_pin_values();
  oldPinValues = pinValues;
}

void loop(){
  /*
   * Read in the state of all zones
   */

   pinValues = read_values();

   /*
    * If there is a change of state display the changed ones.
    */
  if (pinValues != oldPinValues) {
    Serial.print("*Pin value change detected* \r\n");
    display_pin_values();
    oldPinValues = pinValues;
  }
  
  delay(5);
}
