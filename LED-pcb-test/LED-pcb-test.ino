
// 74HC595 RELAY PinOut
const int latchPinOut = 5;     // Connects to the RCLK pin of the 595 (12)  ST_CP
const int clockPinOut = 6;     // Connects to the SRCLK ping of the 595 (11) SH_CP
const int dataPinOut = 7                                                                                                                                                                                                                                                                                                                                                                                                                        ;      // Connects to the SER pin of the 595 (14) DS
const int OEPinOut = 4;        // Connects to OE pin of the 595 (13)

const int readyLED = 2;

int tempVal;
#define NUMSERVOS 4 // Enter the number of servos being controlled

long RELAYPattern = 0;



void set_RELAYS()
{
  digitalWrite(OEPinOut, LOW);
  digitalWrite(latchPinOut, LOW);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, RELAYPattern);
  shiftOut(dataPinOut, clockPinOut, LSBFIRST, (RELAYPattern >> 8));
  digitalWrite(latchPinOut, HIGH);
}

void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("relay-pcb-test....");
  Serial.println("Setting up ....");

  // Ready LED PIN
  pinMode(readyLED, OUTPUT);

  // Setup 74HC595 RELAY serial connections
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);
  digitalWrite(OEPinOut, HIGH);
  pinMode(OEPinOut, OUTPUT);

}

void loop()
{

  // Light readyLED
  digitalWrite(readyLED, HIGH);
  RELAYPattern = 0b1111111111111111;
  set_RELAYS();
  Serial.println("On");
  delay (1000);
  digitalWrite(readyLED, LOW);
  RELAYPattern = 0b0000000000000000;
  set_RELAYS();
  Serial.println("off");
  delay (1000);


}
