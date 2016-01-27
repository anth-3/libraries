/******************************************************************************
* File: DHT.cpp                                              Garduino Library *
* Description: Code to handle a DHTx type humidity and temperature sensor,    *
*     for use in the larger Garduino garden sensor project.                   *
*                                                                             *
* Copyright (c) 2016 At the Lamppost. All rights reserved.                    *
* Written by Trey Holdener <trey@atthelamppost.net>                           *
* MIT License (see LICENSE.txt)                                               *
******************************************************************************/

#include <DHT.h>

DHT::DHT(void) {
    _maxCycles = microsecondsToClockCycles(DHT_MAX_CYCLES);
    _lastTemperature = _lastHumidity = 0;
    ADD_BIT(_status, STATUS_TEMP_GOOD | STATUS_HUMI_GOOD);

    DEBUG_PRINTLN("DHT loaded.");
}

void DHT::begin(uint8_t pin, uint8_t type) {
   _pin = pin;
    _type = type;
    #ifdef __AVR
        _bit = digitalPinToBitMask(pin);
        _port = digitalPinToPort(pin);
    #endif
     /* Set up the pins */
    pinMode(_pin, INPUT_PULLUP);
    /*
     * Using this value makes sure that millis() - lastReadTime will be >= 
     * DHT_MIN_INTERVAL right away. Note that this assignment wraps around,
     * and so will be the subtraction.
     */
    _lastReadTime = -DHT_MIN_INTERVAL;
    DEBUG_PRINT("Max clock cycles: ");
    DEBUG_PRINTLN(_maxCycles, DEC);
}

float DHT::convertC2F(float c) {
    return (c * 1.8 + 32);
}

float DHT::convertF2C(float f) {
    return ((f - 32) * 0.55555);
}

float DHT::computeHeatIndex(float temperature, float percentHumidity, bool isTemperatureFahrenheit) {
    /* 
     * Using both Rothfusz and Steadman's equations
     * http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
     */
    float hi;

    if (!isTemperatureFahrenheit) {
        temperature = convertC2F(temperature);
    }

    hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

    if (hi > 79) {
        hi = 42.379 + (2.04901523 * temperature) + (10.14333127 * percentHumidity) + 
                (-0.22475541 * temperature * percentHumidity) + (-0.00683783 * pow(temperature, 2)) + 
                (-0.05481717 * pow(percentHumidity, 2)) + (0.00122874 * pow(temperature, 2) * percentHumidity) +
                (0.0085282 * temperature * pow(percentHumidity, 2)) + 
                (-0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2));

        if ((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0)) {
            hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);
        } else if ((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0)) {
            hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
        }
    }

    return (isTemperatureFahrenheit ? hi : convertF2C(hi));
}

float DHT::readTemperature(bool isTemperatureFahrenheit, bool force) {
    float f = NAN;

    if (read(force)) {
        switch (_type) {
            case DHT11:
                f = data[2];
            case DHT21:
            case DHT22:
                f = data[2] & 0x7F;
                f *= 256;
                f += 0.1;
                
                /* Negative temperature */
                if (data[2] & 0x80) {
                    f *= -1;
                }
                break;
        }
    }

    _lastTemperature = isTemperatureFahrenheit ? convertC2F(f) : f;

    return (isTemperatureFahrenheit ? convertC2F(f) : f);
}

float DHT::readHumidity(bool force) {
    float f = NAN;

    if (read()) {
        switch (_type) {
            case DHT11:
                f = data[0];
                break;
            case DHT21:
            case DHT22:
                f = data[0];
                f *= 256;
                f += data[1];
                f *= 0.1;
                break;
        }
    }

    _lastHumidity = f;

    return (f);
}

bool DHT::read(bool force) {
    /* 
     * Check if the sensor was read less than a second ago and return early 
     * to use the last reading taken.
     */
    uint32_t currentTime = millis();

    if (!force && ((currentTime - _lastReadTime) < DHT_MIN_INTERVAL)) {
        return _lastResult;
    }

    _lastReadTime = currentTime;

    /* Reset 40 bits of received data to zero */
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    /* 
     * Go into high impedence state to let pull-up raise data line level and 
     * start the reading process.
     */
    digitalWrite(_pin, HIGH);
    delay(250);

    /* First set the data line low for 20 ms */
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delay(20);

    uint32_t cycles[80];

    /* * * * * * * * * * * Timing-critical code * * * * * * * * * * */
    {
        /* 
         * Turn off interrupts temporarily because the next sections are 
         * timing-critical and we do not want any interruptions.
         */
        InterruptLock lock;

        /* End the start signal by setting the data line high for 40ms */
        digitalWrite(_pin, HIGH);
        delayMicroseconds(40);

        /* 
         * Now start reading the data line to get the value from the DHT 
         * sensor. 
         */
        pinMode(_pin, INPUT_PULLUP);
        delayMicroseconds(10); // Delay a bit to let the sensor pull the data line low

        /* 
         * First, expect a low signal for ~80 ms followed by a high signal for 
         * ~80 ms.
         */
        if (expectPulse(LOW) == 0) {
            DEBUG_PRINTLN(F("Timeout waiting for start signal low pulse."));
            _lastResult = false;

            return (_lastResult);
        }
        if (expectPulse(HIGH) == 0) {
            DEBUG_PRINTLN(F("Timeout waiting for start signal high pulse."));
            _lastResult = false;

            return (_lastResult);
        }

        /* 
         * Now read the 40 bits sent by the sensor.  Each bit is sent as a 
         * 50 ms low pulse followed by a variable length high pulse.  If the
         * high pulse is ~28 ms then it is a 0 and if it is ~70 ms then it is 
         * a 1.  We measure the cycle count of the initial 50us low pulse and
         * use that to compare to the cycle count of the high pulse to 
         * determine if the bit is a 0 (high state cycle count < low state 
         * cycle count), or a 1 (high state cycle count > low state cycle 
         * count). Note that for speed all the pulses are read into a array and 
         * then examined in a later step.
         */
        for (int i = 0; i < 80; i += 2) {
            cycles[i] = expectPulse(LOW);
            cycles[i + 1] = expectPulse(HIGH);
        }
    }

    /* 
     * Inspect pulses and determine which ones are 0 (high state cycle count < 
     * low state cycle count), or 1 (high state cycle count > low state cycle 
     * count).
     */
    for (int i = 0; i < 40; ++i) {
        uint32_t lowCycles = cycles[2 * i];
        uint32_t highCycles = cycles[2 * i + 1];

        if ((lowCycles == 0) || (highCycles == 0)) {
            DEBUG_PRINTLN(F("Timeout waiting for pulse."));
            _lastResult = false;
    
            return (_lastResult);
        }
        data[i / 8] <<= 1;
        /* 
         * Now compare the low and high cycle times to see if the bit is a 0 or 
         * 1.
         */
        if (highCycles > lowCycles) {
            /* High cycles are greater than 50us low cycle count, must be a 1. */
            data[i / 8] |= 1;
        }
        /* 
         * Else high cycles are less than (or equal to, a weird case) the 50us 
         * low cycle count so this must be a zero.  Nothing needs to be changed 
         * in the stored data.
         */
    }

    DEBUG_PRINTLN(F("Received:"));
    DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", "));
    DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", "));
    DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", "));
    DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", "));
    DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? "));
    DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

    // Check we read 40 bits and that the checksum matches.
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        _lastResult = true;
    
        return (_lastResult);
    } else {
        DEBUG_PRINTLN(F("Checksum failure!"));
        _lastResult = false;

        return (_lastResult);
    }
}

/* 
 * Expect the signal line to be at the specified level for a period of time and
 * return a count of loop cycles spent at that level (this cycle count can be
 * used to compare the relative time of two pulses).  If more than 1 ms 
 * ellapses without the level changing then the call fails with a 0 response.
 * This is adapted from Arduino's pulseInLong function (which is only available
 * in the very latest IDE versions):
 *     https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
 */
uint32_t DHT::expectPulse(bool level) {
    uint32_t count = 0;
    /* 
     * On AVR platforms use direct GPIO port access as it is much faster and 
     * better for catching pulses that are 10's of microseconds in length:
     */
    #ifdef __AVR
        uint8_t portState = level ? _bit : 0;
    
        while ((*portInputRegister(_port) & _bit) == portState) {
            if (count++ >= _maxCycles) {
                return (0); // Exceeded timeout, fail.
            }
        }
    /* 
     * Otherwise fall back to using digitalRead (this seems to be necessary on 
     * ESP8266 right now, perhaps bugs in direct port access functions?).
     */
    #else
        while (digitalRead(_pin) == level) {
            if (count++ >= _maxCycles) {
                return (0); // Exceeded timeout, fail.
            }
        }
    #endif

    return (count);
}

uint8_t DHT::getStatus(void) {
    // If temperature is at an extreme, set to warning...
    if (_lastTemperature > 100 || _lastTemperature < 60) {
        REMOVE_BIT(_status, STATUS_TEMP_GOOD);
        ADD_BIT(_status, STATUS_TEMP_WARN);
        // If temperature is really extreme, set to alert...
        if (_lastTemperature > 110 || _lastTemperature < 40) {
            REMOVE_BIT(_status, STATUS_TEMP_WARN);
            ADD_BIT(_status, STATUS_TEMP_ALRT);
        }
    }

    // If humidity is low, set to warning...
    if (_lastHumidity < 60) {
        REMOVE_BIT(_status, STATUS_HUMI_GOOD);
        ADD_BIT(_status, STATUS_HUMI_WARN);
        // If humidity is extreme, set to alert...
        if (_lastHumidity < 20) {
            REMOVE_BIT(_status, STATUS_HUMI_WARN);
            ADD_BIT(_status, STATUS_HUMI_ALRT);
        }
    }

    return (_status);
}
