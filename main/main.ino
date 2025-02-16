/*
 * Ronit Kumar - AgroSense, low-cost tool for capturing data of various environmental varible which impacts crops growth.
 * 
 * -- 17/02/2025--
 * 
 * Utilizes the following sensors:
 * 
 *  - BMP_280  (I2C) : Monitors temperature, altitude and pressure around the crops.
 * 
 * Benchmarking
 *  - SHT_31   (I2C) : Monitors temperature and humidity around crops.
 *  - DHT_22         : Monitors temperature and moisture around the crops.
 * 
 * Benchmarking 
 *  - MQ_135         : Monitors CO2 levels around the crops (utilizes reactions).
 *  - NDIR_CO2       : Monitors CO2 levels around the crops (utilizes Non-Dispersive Infrared).
 * 
 * Progress Tracking
 *  - Camera         : Capture crop photograph to relate it with crops growth, i.e. measure correlation among them.
 *  - SD Card Module : Storing images locally.
 * 
 */

/* Header files*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <MQ135.h>
#include "Adafruit_SHT31.h"
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define DHTPIN 2
#define DHTTYPE DHT22
#define PIN_MQ135 4


Adafruit_BMP280 bmp; // I2C
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // I2C
DHT dht(DHTPIN, DHTTYPE);
MQ135 mq135_sensor(PIN_MQ135);

bool enableHeater = false;
uint8_t loopCnt = 0;

/* 
 * BMP280 internal helper functions
 */
void bmp_280_status_check()
{
    /* BMP280 init and status check */
    Serial.println(F("BMP280 test"));
    unsigned status;

    status = bmp.begin(0x76);
    if (!status) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
}

void bmp_280_get_env_data(float &temperature, float &pressure, float &altitude)
{
   /* BMP280 reading */
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1013.25); /* Adjusted to local forecast */

    Serial.print(F("BMP280 Temperature = "));
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print(F("BMP280 Pressure = "));
    Serial.print(pressure);
    Serial.println(" Pa");

    Serial.print(F("BMP280 Approx altitude = "));
    Serial.print(altitude);
    Serial.println(" m");
}

/* 
 * SHT31 internal helper functions
 */
void sht_31_status_check()
{
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

void sht_31_get_env_data(float &temperature, float &humidity)
{
    /* SHT31 reading */
    temperature = sht31.readTemperature();
    humidity = sht31.readHumidity();

    if (!isnan(temperature)) {
        Serial.print("Temp *C = ");
        Serial.print(temperature);
        Serial.print("\t\t");
    } else {
        Serial.println("Failed to read temperature");
    }

    if (!isnan(humidity)) {
        Serial.print("Hum. % = ");
        Serial.println(humidity);
    } else {
        Serial.println("Failed to read humidity");
    }

    delay(1000);

    // Toggle heater enabled state every 30 seconds
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

/* 
 * DHT22 internal helper functions
 */
void dht_22_get_env_data(float &temperature, float &humidity, float &hif)
{
    // Wait a few seconds between measurements.
    delay(2000);

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temperature = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature) || isnan(f)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Compute heat index in Fahrenheit (the default)
    hif = dht.computeHeatIndex(f, humidity);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(temperature, humidity, false);

    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.print(F("%  Temperature: "));
    Serial.print(temperature);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));
    Serial.print(hif);
    Serial.println(F("°F"));
}

/* 
 * DHT22 internal helper functions
 */
void mq_135_get_env_data(float &ppm)
{
    ppm = mq135_sensor.getPPM();
    Serial.print("\t PPM: ");
    Serial.print(ppm);
    delay(300);

}
/* setup()      : Initializes, load and runs preliminary tests before going into loop() */
void setup()
{
    Serial.begin(9600);
    while(!Serial)
        delay(100);
    
    bmp_280_status_check();
    sht_31_status_check();
    dht.begin();
}

/* loop()       : goes around monitoring and sending data to cloud for further utilization */
void loop()
{
    float bmp_temp, bmp_pressure, bmp_altitude;
    float sht_temp, sht_humidity;
    float dht_temp, dht_humidity, dht_heatIndex;

    /* fetch bmp_280 readings */
    bmp_280_get_env_data(bmp_temp, bmp_pressure, bmp_altitude);

    Serial.println();

    /* fetch sht_31 readings */
    sht_31_get_env_data(sht_temp, sht_humidity);

    Serial.println();

    dht_22_get_env_data(dht_temp, dht_humidity, dht_heatIndex);
}