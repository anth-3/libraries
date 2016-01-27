/******************************************************************************
* File: Garduino.h                                           Garduino Library *
* Description: Header to handle a DHTx type humidity and temperature sensor,  *
*     for use in the larger Garduino garden sensor project.                   *
*                                                                             *
* Copyright (c) 2016 At the Lamppost. All rights reserved.                    *
* Written by Trey Holdener <trey@atthelamppost.net>                           *
* MIT License (see LICENSE.txt)                                               *
******************************************************************************/

#ifndef __GARDUINO_H__

#define __GARDUINO_H__

#if ARDUINO >= 100
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif
#include <LiquidCrystal.h>

#include "Lamppost.h"
#include "DHT.h"
#include "Photosensor.h"
#include "Hygrometer.h"

/* Uncomment the following line to enable debugging on this project */
#define GARDUINO_DEBUG
/* Define where the debug printing will be output */
#define DEBUG_PRINTER Serial

/* Setup debug print macros */
#ifdef GARDUINO_DEBUG
    #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
    #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
    #define DEBUG_PRINT(...) {}
    #define DEBUG_PRINTLN(...) {}
#endif

#define GARDUINO_VERSION    "v0.1.1"

#define TIME_SECOND     1000
#define TIME_MINUTE     60 * TIME_SECOND
#define TIME_HOUR       60 * TIME_MINUTE
#define TIME_DAY        24 * TIME_HOUR

#define DHT_INTERVAL            5 * TIME_SECOND     // 5 seconds
#define HYGROMETER_INTERVAL     8 * TIME_HOUR       // 8 hours (longer?)
#define PHOTOSENSOR_INTERVAL    15 * TIME_MINUTE    // 15 minutes

#define STATUS_TEMP_ALRT        (1 << 0)
#define STATUS_HUMI_ALRT        (1 << 1)
#define STATUS_HYGR_ALRT        (1 << 2)
#define STATUS_PHOT_ALRT        (1 << 3)

#define STATUS_TEMP_WARN        (1 << 0)
#define STATUS_HUMI_WARN        (1 << 1)
#define STATUS_HYGR_WARN        (1 << 2)
#define STATUS_PHOT_WARN        (1 << 3)

#define STATUS_TEMP_GOOD        (1 << 0)
#define STATUS_HUMI_GOOD        (1 << 1)
#define STATUS_HYGR_GOOD        (1 << 2)
#define STATUS_PHOT_GOOD        (1 << 3)

/* Pins */
#define HYGROMETER_SENSOR_PIN   A0  // The Analog pin the Hygrometer sensor is connected to
#define HYGROMETER_VOLTAGE_PIN  13
#define PHOTOSENSOR_PIN         A1  // The Analog pin the Photosensor is connected to
#define DHT_PIN                 10  // The digital pin the DHT sensor is connected to
#define LCD_RS_PIN              12
#define LCD_ENABLE_PIN          11
#define LCD_D4_PIN              5
#define LCD_D5_PIN              4
#define LCD_D6_PIN              3
#define LCD_D7_PIN              2
#define LED_WORKING_PIN         9
#define LED_WARNING_PIN         6
#define LED_ALERT_PIN           7
#define LED_ALLOK_PIN           8
#define LED_PHOTOSENSOR_PIN     A2  // The Analog pin to display the output of the photosensor

class DHT;
class Photosensor;
class Hygrometer;

class Garduino {
    public:
        Garduino(void);
        void begin(bool hasLCD);
        void pulse(void);
        void resetLCD(void);
    private:
        /* Time values to control interval of sensor checks */
        uint32_t _garduinoTime, _lastTimeCheck;
        DHT *_dht;
        Photosensor *_photosensor;
        Hygrometer *_hygrometer;

        LiquidCrystal *_lcd;
};

#endif
