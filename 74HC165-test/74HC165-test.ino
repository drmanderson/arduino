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

  BYTES_VAL_T bitVal;
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

void move_points() {
  for (int i=0;i< DATA_WIDTH; i++){
    if ((pinValues >> i) & 1) {
      Serial.print("Switch: ");
      Serial.print(i+1);
      Serial.print(" : triggered");
      Serial.print("\r\n");
    }

  }
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
    move_points();
    oldPinValues = pinValues;
  }
  
  delay(200);
}
