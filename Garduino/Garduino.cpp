/******************************************************************************
* File: Garduino.cpp                                         Garduino Library *
* Description: Code that handles the main loops of the project, and utilizes  *
*     the library to query all of the sensors.                                *
*                                                                             *
* Copyright (c) 2016 At the Lamppost. All rights reserved.                    *
* Written by Trey Holdener <trey@atthelamppost.net>                           *
* MIT License (see LICENSE.txt)                                               *
******************************************************************************/

#include "Garduino.h"

/* Uncomment whichever sensor is being used (only one at a time) */
#define DHT_TYPE            DHT11   // DHT 11
//#define DHT_TYPE          DHT21   // DHT 21 (AM2301)
//#define DHT_TYPE          DHT22   // DHT 22 (AM2302, AM2321)

Garduino::Garduino(void) {
    resetLCD();
    _lastTimeCheck = -DHT_INTERVAL;

    Serial.begin(9600);
    DEBUG_PRINT("Garduino "); DEBUG_PRINTLN(GARDUINO_VERSION);
    DEBUG_PRINTLN("Copyright (c) At the Lamppost. All rights reserved.");
    DEBUG_PRINTLN("by Trey Holdener <trey@atthelamppost.net>");
    DEBUG_PRINTLN("MIT License");
}

void Garduino::begin(bool hasLCD) {
    if (hasLCD) {
        _lcd = new LiquidCrystal(LCD_RS_PIN, LCD_ENABLE_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
        #define LCD_CLEAR() { _lcd.clear(); }
        #define LCD_CURSOR(...) { _lcd.setCursor(__VA_ARGS__); }
        #define LCD_PRINT(...) { _lcd.print(__VA_ARGS__); }
    } else {
        #define LCD_CLEAR() {}
        #define LCD_CURSOR(...) {}
        #define LCD_PRINT(...) {}
    }

    LCD_PRINT("Garduino");
    delay(2000);
    LCD_CURSOR(1, 0);
    LCD_PRINT("Loading");
    for (uint8_t i = 0; i < 5; i++) {
        delay(500);
        LCD_PRINT(".");
    }

    _dht->begin(DHT_PIN, DHT_TYPE);
    _photosensor->begin(PHOTOSENSOR_PIN);
    _hygrometer->begin(HYGROMETER_SENSOR_PIN);

}

void Garduino::resetLCD() {
    LCD_CLEAR();
    LCD_CURSOR(0, 0);
}

void Garduino::pulse() {
    delay(2000); // Wait a few seconds between pulses of the loop

    _garduinoTime = millis();

    if ((_garduinoTime - _lastTimeCheck) >= DHT_INTERVAL) {
        /* 
         * Reading temperature or humidity takes about 250 milliseconds! Sensor 
         * readings may also be up to 2 seconds 'old' (it is a very slow 
         * sensor).
         */
        float h = _dht->readHumidity();
        float f = _dht->readTemperature();

        /* Check if any reads failed and exit early (to try again). */
        if (isnan(h) || isnan(f)) {
            DEBUG_PRINTLN("Failed to read from DHT sensor!");

            return;
        }

        /* Compute heat index */
        float hi = _dht->computeHeatIndex(f, h);

        DEBUG_PRINT("Humidity: "); DEBUG_PRINT(h); DEBUG_PRINT(" %\t");
        DEBUG_PRINT("Temperature: "); DEBUG_PRINT(f); DEBUG_PRINT(" *F\t");
        DEBUG_PRINT("Heat index: "); DEBUG_PRINT(hi); DEBUG_PRINTLN(" *F");

        /* TODO: create proper output */
        LCD_PRINT("H: ");
    } else if ((_garduinoTime - _lastTimeCheck) >= PHOTOSENSOR_INTERVAL) {
        /* Read from the photosensor.  */
        uint8_t l = _photosensor->readPhotocell();

        /* Check if the read failed and exit early (to try again). */
        if (isnan(l)) {
            DEBUG_PRINTLN("Failed to read from Photosemsor!");

            return;
        }

        short ledBrightness = _photosensor->processReading();
        analogWrite(LED_PHOTOSENSOR_PIN, ledBrightness);
    } else if ((_garduinoTime - _lastTimeCheck) >= HYGROMETER_INTERVAL) {
        uint8_t s = _hygrometer->readHygrometer();
    }

    uint8_t dhtStatus, hygroStatus, photoStatus;

    
}
