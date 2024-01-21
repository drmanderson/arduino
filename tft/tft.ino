



// Grafiktest 1.8 Zoll TFT-Farb-Display (HY-1.8 SPI)

#define TFT_PIN_CS   10 // Arduino-Pin an Display CS   
#define TFT_PIN_DC   9  // Arduino-Pin an 
#define TFT_PIN_RST  8  // Arduino Reset-Pin

#include <SPI.h>
#include <Adafruit_GFX.h>    // Adafruit Grafik-Bibliothek
#include <Adafruit_ST7735.h> // Adafruit ST7735-Bibliothek
#include <Wire.h>



Adafruit_ST7735 tft = Adafruit_ST7735(TFT_PIN_CS, TFT_PIN_DC, TFT_PIN_RST);  // Display-Bibliothek Setup
#define SEALEVELPRESSURE_HPA (1013.25)

void setup(void) {
  Serial.begin(115200);
  Serial.println(F("TFT test"));
  tft.initR(INITR_BLACKTAB);   // ST7735-Chip initialisieren


  tft.fillScreen(ST7735_BLACK);
}

void loop() {

  int Temp = 24;
  int Humidity = 60;
  int Hpa = 1023;

  tft.setTextSize(1);
  tft.setTextWrap(true); 
  tft.setCursor(40,4);

  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.println("Weather !");
  tft.drawLine(1,15,125,15,ST7735_GREEN);

  tft.setCursor(4,25);
  tft.print("Temp: ");
  tft.print(Temp);
  tft.println(" C");
  tft.setCursor(4,40);
  tft.print("Pres: ");
  tft.print(Hpa);
  tft.println(" hPa");
  tft.setCursor(4,55);
  tft.print("Hum: ");
  tft.print(Humidity);
  tft.println(" %");

  tft.drawLine(2,75,125,75,ST7735_RED);

  tft.setCursor(10,71);
  tft.print(" T ");

  delay(2000);
}


