



// Grafiktest 1.8 Zoll TFT-Farb-Display (HY-1.8 SPI)

#define TFT_PIN_CS   10 // Arduino-Pin an Display CS   
#define TFT_PIN_DC   9  // Arduino-Pin an 
#define TFT_PIN_RST  8  // Arduino Reset-Pin

#include <SPI.h>
#include <Adafruit_GFX.h>    // Adafruit Grafik-Bibliothek
#include <Adafruit_ST7735.h> // Adafruit ST7735-Bibliothek
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_PIN_CS, TFT_PIN_DC, TFT_PIN_RST);  // Display-Bibliothek Setup
Adafruit_BME280 bme; // use I2C interface
#define SEALEVELPRESSURE_HPA (1013.25)

void setup(void) {
  Serial.begin(115200);
  Serial.println(F("BMP280 test"));
  tft.initR(INITR_BLACKTAB);   // ST7735-Chip initialisieren

  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  tft.fillScreen(ST7735_BLACK);
}

void loop() {

  int Temp = bme.readTemperature();
  int Humidity = bme.readHumidity();
  int Hpa = bme.readPressure() /100.0F;

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


