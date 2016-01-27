/******************************************************************************
* File: DHT.h                                                Garduino Library *
* Description: Header to handle a DHTx type humidity and temperature sensor,  *
*     for use in the larger Garduino garden sensor project.                   *
*                                                                             *
* Copyright (c) 2016 At the Lamppost. All rights reserved.                    *
* Written by Trey Holdener <trey@atthelamppost.net>                           *
* MIT License (see LICENSE.txt)                                               *
******************************************************************************/

#ifndef __DHT_H__

#define __DHT_H__

#include "Garduino.h"

/* Define the types of sensors */
#define DHT11   11
#define DHT21   21
#define DHT22   22

#define DHT_MIN_INTERVAL     2000
#define DHT_MAX_CYCLES       1000 // 1 ms timeout for reading pulses from the DHT sensor

class DHT {
    public:
        DHT(void);
        void begin(uint8_t pin, uint8_t type);
        float convertC2F(float);
        float convertF2C(float);
        float computeHeatIndex(float temperature, float percentHumidity, bool isTemperatureFahrenheit = true);
        float readTemperature(bool isTemperatureFahrenheit = true, bool force = false);
        float readHumidity(bool force = false);
        bool read(bool force = false);
        uint8_t getStatus(void);

    private:
        uint8_t data[5];
        uint8_t _pin, _type;
        uint8_t _status;
        #ifdef __AVR
            /* 
             * Use direct GPIO access on an 8-bit AVR so keep track of the port 
             * and bitmask for the digital pin connected to the DHT.  Other 
             * platforms will use digitalRead.
             */
            uint8_t _bit, _port;
        #endif
        uint32_t _lastReadTime, _maxCycles;
        bool _lastResult;
        uint8_t _lastTemperature, _lastHumidity;

        uint32_t expectPulse(bool level);
};

class InterruptLock {
    public:
        InterruptLock() {
            noInterrupts();
        }
        ~InterruptLock() {
            interrupts();
        }
};

#endif
