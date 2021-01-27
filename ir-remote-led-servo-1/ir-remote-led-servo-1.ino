#include <IRremote.h>
#include <Servo.h>
Servo myservo;
const int IRpin = 11;
int servopin = 9;
int pos0 = 0;
int pos1 = 180; 
int pos2 = 90;

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
  myservo.attach(servopin);  // attaches the servo on pin 9 to the servo object  
  myservo.write(pos0);  
}

void loop() {
  if (irrecv.decode(&results)) { 
    Serial.println(results.value, HEX ); // Print the Serial 'results.value'

    switch (results.value) {
      case 0x9716BE3F:
        digitalWrite(redPin, LOW);
        digitalWrite(greenPin, HIGH);
        digitalWrite(bluePin, LOW);
        myservo.write(pos1);        
        break;

      case 0x3D9AE3F7:
        digitalWrite(bluePin, LOW);
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);
        myservo.write(pos2);
        break;

      case 0x6182021B:
        digitalWrite(greenPin, LOW);
        digitalWrite(redPin, LOW);
        digitalWrite(bluePin, HIGH);
        myservo.write(pos0);   
        break;

      case 0xC101E57B:
        digitalWrite(redPin, LOW);
        digitalWrite(bluePin, LOW);  
        digitalWrite(greenPin, LOW); 
        myservo.write(pos0);     
        break;

      case 0x3EC3FC1B:
        sequence();
    }
    irrecv.resume();   // Receive the next value
  }
}

void sequence() {
  digitalWrite (redPin, LOW);
  digitalWrite (greenPin, HIGH);
  digitalWrite (bluePin, LOW);
  myservo.write(pos2);
  delay(250);
  digitalWrite (redPin, HIGH);
  digitalWrite (greenPin, LOW);
  digitalWrite (bluePin, LOW);
  myservo.write(pos1);
  delay(250);
  digitalWrite (redPin, LOW);
  digitalWrite (greenPin, LOW);
  digitalWrite (bluePin, HIGH);
  myservo.write(pos0);
  delay(250);
  digitalWrite (redPin, LOW);
  digitalWrite (greenPin, LOW);
  digitalWrite (bluePin, LOW);
  delay(100);  
}
