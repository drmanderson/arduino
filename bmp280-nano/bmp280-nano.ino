

/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
//#include <Adafruit_BMP280.h>
// include the library code:
//#include <LiquidCrystal.h>

#define BMP_SCK  (24)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (23)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);


// initialize the library with the numbers of the interface pins
const int rs = 5, en = 6, d4 = 7, d5 = 8, d6 = 9, d7 = 10;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*
 10 - LCD D7
 9 - LCD D6
 8 - LCD D5
 7 - LCD D4
 6 - LCD E(nable)
 5 - LCD RS
 LCD R/W pin to ground
 LCD VSS pin to ground
 LCD VCC pin to 5V
 10K resistor:
 ends to +5V and ground
 wiper to LCD VO pin (pin 3)
*/

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // Setup LCD screen
 // lcd.begin(16,2); 
 // lcd.print("Setup....");
}

void loop() {
    // lcd_clear_line (1);
    // lcd_clear_line (2);
    // lcd.setCursor(0,0);
    int temp = bmp.readTemperature();
   Serial.print(F("Temperature = "));
   Serial.print(temp);
   Serial.println(" *C");
    // lcd.print("T: ");
    // lcd.print(temp); 
    // lcd.print(" C. ");
    float pres = (bmp.readPressure());
   Serial.print(F("Pressure = "));
   Serial.print(pres/100);
   Serial.println(" Pa");
    // lcd.setCursor(0,1);
    // lcd.print("P: ");
    // lcd.print(pres/100);
    // lcd.print(" hPa");
    
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1020)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(2000);
}

// void lcd_clear_line (int line) {
//   // clear line on lcd display 
//   lcd.setCursor(0,line);
//   lcd.print("                                                                   ");
// }
