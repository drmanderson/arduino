#include <IRremote.h>
const int IRpin = 11;

IRrecv irrecv(IRpin);
decode_results results;

const int redPin = 3;
const int greenPin = 2;
const int bluePin = 4;



void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);

  pinMode (redPin, OUTPUT);
  pinMode (greenPin, OUTPUT);
  pinMode (bluePin, OUTPUT);

  sequence ();

}

void loop() {
  if (irrecv.decode(&results)) { 
    Serial.println(results.value, HEX ); // Print the Serial 'results.value'

    switch (results.value) {
      case 0x9716BE3F:
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);
        digitalWrite(bluePin, LOW);
        break;

      case 0x3D9AE3F7:
        digitalWrite(bluePin, HIGH);
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, LOW);
        break;

      case 0x6182021B:
        digitalWrite(greenPin, HIGH);
        digitalWrite(redPin, LOW);
        digitalWrite(bluePin, LOW);
        break;

      case 0xC101E57B:
        digitalWrite(redPin, LOW);
        digitalWrite(bluePin, LOW);  
        digitalWrite(greenPin, LOW);    
        break;

      case 0x3EC3FC1B:
        sequence();
    }
    irrecv.resume();   // Receive the next value
  }
}

void sequence() {
  digitalWrite (redPin, HIGH);
  digitalWrite (greenPin, LOW);
  digitalWrite (bluePin, LOW);
  delay(250);
  digitalWrite (redPin, LOW);
  digitalWrite (greenPin, HIGH);
  digitalWrite (bluePin, LOW);
  delay(250);
  digitalWrite (redPin, LOW);
  digitalWrite (greenPin, LOW);
  digitalWrite (bluePin, HIGH);
  delay(250);
  digitalWrite (redPin, LOW);
  digitalWrite (greenPin, LOW);
  digitalWrite (bluePin, LOW);
  delay(100);  
}
