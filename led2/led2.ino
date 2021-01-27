// This is the two digit multiplexed display demo.

int segPins[] = {2, 3, 4, 5, 6, 8, 9, 7 }; // (a,b,c,d,e,f,g.dt)
int displayPins[] = {10, 11};       // pin 10 controls D0, pin 11 controls D1
int displayBuf[2];                  // The display buffer contains the digits to be displayed.
                                    // displayBuf[0] contains the LSD, displayBuf[1] contains the MSD

byte segCode[10][8] = {
// 7 segment code table
//  a  b  c  d  e  f  g  .
  { 1, 1, 1, 1, 1, 1, 0, 0},  // 0
  { 0, 1, 1, 0, 0, 0, 0, 0},  // 1
  { 1, 1, 0, 1, 1, 0, 1, 0},  // 2
  { 1, 1, 1, 1, 0, 0, 1, 0},  // 3
  { 0, 1, 1, 0, 0, 1, 1, 0},  // 4
  { 1, 0, 1, 1, 0, 1, 1, 0},  // 5
  { 1, 0, 1, 1, 1, 1, 1, 0},  // 6
  { 1, 1, 1, 0, 0, 0, 0, 0},  // 7
  { 1, 1, 1, 1, 1, 1, 1, 0},  // 8
  { 1, 1, 1, 1, 0, 1, 1, 0}   // 9
};


void refreshDisplay(int digit1, int digit0)
{
  digitalWrite(displayPins[0], HIGH);  // displays digit 0 (least significant)
  digitalWrite(displayPins[1], LOW );
  setSegments(digit0);
  delay(5);
  digitalWrite(displayPins[0], LOW);    // then displays digit 1
  digitalWrite(displayPins[1], HIGH);
  setSegments(digit1);
  delay(5);
}

void setSegments(int n)
{
  for (int i=0; i < 8; i++)
  {
    digitalWrite(segPins[i], segCode[n][i]);
  }
}

//***************************************************************************
//******************************************************************************
void setup()
{
  for (int i=0; i < 8; i++)
  {
    pinMode(segPins[i], OUTPUT);
  }
  pinMode(displayPins[0], OUTPUT);
  pinMode(displayPins[1], OUTPUT);

  displayBuf[1] = 0;    // initializes the display
  displayBuf[0] = 0;
   }

int i = 0, j = 0;
int startTime = 0;
int endTime;
//******************************************************************************
//******************************************************************************
void loop()
{
    refreshDisplay(displayBuf[1],displayBuf[0]);  // Refreshes the display with the contents of displayBuf
                                                     // each iteration of the main loop.


   endTime = millis();                  // increments the counter approximately once a second
   if ((endTime - startTime) >= 1000)
   {
      if (++i > 9)
      {
        i = 0;
        if (++j > 9) j = 0;
      }
      displayBuf[0] = i;                // send the updated count to the display buffer
      displayBuf[1] = j;
      startTime = endTime;
   }

}
