/*
 *  Ronit Kumar - AgroSense, low-cost tool for capturing data of various environmental varible which impacts crops growth.
 * 
 *  Utilizes the following sensors:
 * 
 *  - BMP280    : Monitors temperature, altitude and pressure around the crops.
 *  - SHT31     : Monitors temperature and humidity around crops.
 *  - K30 CO2   : Monitors CO2 levels around the crops
 * 
 *  - Camera    : Capture crop photograph to relate it with crops growth, i.e. measure correlation among them.
 * 
 */

/* Header files*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // I2C

bool enableHeater = false;
uint8_t loopCnt = 0;

/* BMP280 internal helper functions */


/* SHT31 internal helper functions*/

/* setup()      : Initializes, load and runs preliminary tests before going into loop() */
void setup()
{
    Serial.begin(9600);
    while(!Serial)
        delay(100);
    
    /* BMP280 init and status check */
    Serial.println(F("BMP280 test"));
    unsigned status;

    status = bmp.begin();
    if (!status) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

    /* SHT31 init and status check */
    Serial.println("SHT31 test");
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        Serial.println("Couldn't find SHT31");
        while (1) delay(1);
    }

    Serial.print("Heater Enabled State: ");
    if (sht31.isHeaterEnabled())
        Serial.println("ENABLED");
    else
        Serial.println("DISABLED");
}

/* loop()       : goes around monitoring and sending data to cloud for further utilization */
void loop()
{
    /* BMP280 reading over serial monitor */
    Serial.print(F("BMP280 Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("BMP280 Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("BMP280 Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println("\n\n\n");

    /* SHT31 reading over serial monitor */
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    if (! isnan(t)) {  // check if 'is not a number'
        Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
    } else { 
        Serial.println("Failed to read temperature");
    }
  
    if (! isnan(h)) {  // check if 'is not a number'
        Serial.print("Hum. % = "); Serial.println(h);
    } else { 
        Serial.println("Failed to read humidity");
    }

    delay(1000);

    // Toggle heater enabled state every 30 seconds
    // An ~3.0 degC temperature increase can be noted when heater is enabled
    if (loopCnt >= 30) {
        enableHeater = !enableHeater;
        sht31.heater(enableHeater);
        Serial.print("Heater Enabled State: ");
        if (sht31.isHeaterEnabled())
            Serial.println("ENABLED");
        else
            Serial.println("DISABLED");

        loopCnt = 0;
    }
    loopCnt++;

}