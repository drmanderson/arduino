/* How to use the DHT-22 sensor with Arduino uno
   Temperature and humidity sensor
*/

//Libraries
#include <DHT.h>
// include the library code:
#include <LiquidCrystal.h>

//Constants
#define DHTPIN 8     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7 , 6, 5, 4, 3, 2);
/*
 2 - LCD D7
 3 - LCD D6
 4 - LCD D5
 5 - LCD D4
 6 - LCD E(nable)
 7 - LCD RS
 LCD R/W pin to ground
 LCD VSS pin to ground
 LCD VCC pin to 5V
 10K resistor:
 ends to +5V and ground
 wiper to LCD VO pin (pin 3)
*/

//Variables
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value

void setup()
{
  Serial.begin(9600);
  dht.begin();
  // Setup LCD screen
  lcd.begin(16,2); 
}

void loop()
{
    delay(2000); //Delay 2 sec.
    //Read data and store it to variables hum and temp
    hum = dht.readHumidity();
    temp = dht.readTemperature();
    //Print temp and humidity values to serial monitor
    lcd.setCursor(0,0);
    lcd.print("Humidity: ");
    lcd.print(hum);
    lcd.print("%: ");
    lcd.setCursor(0,1);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print(" C");
}

   
