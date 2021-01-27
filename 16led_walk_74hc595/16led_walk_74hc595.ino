

// Demonstration code for 74HC595 16 bit output
int RCLKPin = 9;   // pin 12 on the 74hc595 latch - nSS
int SRCLKPin = 10;  // pin 11 on the 74hc595 shift register clock - SCK
int SERPin = 11;    // pin 14 on the 74hc595 data - MOSI
unsigned int d;    // Data to be sent to the shift reg.
int dir =0;        // Direction of walking 1.
char buf[12];      // General purpose buffer.

void setup() {
  Serial.begin(9600);          // start serial port (debug).

  pinMode(RCLKPin, OUTPUT);    // Set 595 control PIN sto output.
  pinMode(SRCLKPin, OUTPUT);
  pinMode(SERPin, OUTPUT);

  Serial.println("74HC595 Demo 2xchips for 16 bit register.");

  d=1;
}

void loop() {
  delay(85);
  Serial.println("============================");
  do_loop();

}


void do_loop() {
    delay(100);
    digitalWrite(RCLKPin, LOW);
    Serial.print("d is : ");

    shiftOut(SERPin, SRCLKPin, MSBFIRST, (0xff00 & d)>>8);
    shiftOut(SERPin, SRCLKPin, MSBFIRST, 0x00ff & d);
    digitalWrite(RCLKPin, HIGH);


    if (!dir){
      d<<=1;
      Serial.print(d,BIN);
      Serial.print(", 0x");
      Serial.print(d,HEX);
      Serial.print(", ");
      Serial.println(d);
    }  else {
      d>>=1; // Shift
      Serial.print(d,BIN);
      Serial.print(", 0x");
      Serial.print(d,HEX);
      Serial.print(", ");
      Serial.println(d);
    }

    if (d&0x8000) {
      dir=1;           // Set direction.
      Serial.print("0x");
      Serial.print(d,HEX);
      Serial.print(", change dir 0->1, d&0x8000=");
      Serial.println (d&0x8000);
    }
    if (d&0x0001){
      dir=0;
      Serial.print("0x");
      Serial.print(d,HEX);
      Serial.print(", change dir 1->0, d&0x0001="); 
      Serial.println(d&0x0001); 
    }
}
