int latchPin = 12;
int dataPin = 411;
int clockPin = 13;

const float segSpeed = .05; //How quickly to flash each led segment.
const int digLoop = 100; //How many times to repeat the segment sequence.
const int nextPin = 200; //Delay before moving to the next digit on the display.
const int repeatSeq = 300; //Delay before starting over from the first digit.
const int pinArray[4] = {3, 5}; //Defines PWM pins, the pins that activate each digiti.
const int digArray[11][9] = {
  {127, 191, 223, 239, 247, 251, 255, 254, 255}, //0: 127 top led segment
  {255, 191, 223, 255, 255, 255, 255, 254, 255}, //1: 191 top right seg
  {127, 191, 255, 239, 247, 255, 253, 254, 255}, //2: 255 is NO segment
  {127, 191, 223, 239, 255, 255, 253, 254, 255}, //3: 223 bottom right seg
  {255, 191, 223, 255, 255, 251, 253, 254, 255}, //4: 
  {127, 255, 223, 239, 255, 251, 253, 254, 255}, //5: 239 bottom seg
  {127, 191, 223, 239, 247, 255, 253, 254, 255}, //6: 247 bottom left seg
  {127, 191, 223, 255, 255, 255, 255, 254, 255}, //7: 
  /*Example for the number 7:
   * 127:   |_ON  |
   * 251:OFF    ON:191
   * 253:   OFF | |
   * 247:OFF    ON:223
   * 239:   OFF |_|  
   */
  {127, 191, 223, 239, 247, 251, 253, 254, 255}, //8: 251 top left seg
  {127, 191, 223, 239, 255, 251, 253, 254, 255}, //9: 253 center seg
  {255, 255, 255, 255, 255, 255, 255, 254, 255}, //DP: Decimal Point seg
};
const int arrHeight = 9; // (0-9) 9 displays digits 0-9. 
const int arrWidth = 6; // (0-8) how many segments from each row to use. 7 will add the decimal row.

void setup() {
  //Serial.begin(9600); //which port to output serial display data. unused. slows display down.
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
}

void loop() {
  for (int i = 0; i <= 3; i++) { //cycles through all PWM pins
    for (int k = 0; k <= arrHeight; k++) { //cycles through countTo number of digArray rows
      analogWrite((pinArray[i]), 255); //turns PWM pin(i) on
      for (int h = 0; h <= digLoop; h++) { //how many times to cycle through columns
        for (int j = 0; j <= arrWidth; j++) { //cycles through all displayed column data
          digitalWrite(latchPin, HIGH);
          shiftOut(dataPin, clockPin, LSBFIRST, digArray[k][j]);
          digitalWrite(latchPin, LOW);
          delay(segSpeed); //delay until next segment displayed sequence
        }
      }
      analogWrite((pinArray[i]), 0); //turns PWM pin(i) off
      delay(nextPin); //delay until next displayed digit sequence
    }
  }
  delay(repeatSeq); //delay until everything is started over again
}
