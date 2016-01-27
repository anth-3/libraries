/******************************************************************************
* File: Photosensor.h                                        Garduino Library *
* Description: Code to handle a photosensor, detecting light amounts, for use *
*     in the larger Garduino garden sensor project.                           *
*                                                                             *
* Copyright (c) 2016 At the Lamppost. All rights reserved.                    *
* Written by Trey Holdener <trey@atthelamppost.net>                           *
* MIT License (see LICENSE.txt)                                               *
******************************************************************************/

#include "Photosensor.h"

Photosensor::Photosensor(void) {
    ADD_BIT(_status, STATUS_PHOT_GOOD);

    DEBUG_PRINTLN("Photosensor loaded.");
}

void Photosensor::begin(uint8_t pin) {
    _pin = pin;

    /* Set up the pins */
    pinMode(_pin, INPUT_PULLUP);
    _insufficientLight = false;
}

uint8_t Photosensor::readPhotocell() {
    _lastReading = analogRead(_pin);

    return (_lastReading);
}

uint8_t Photosensor::processReading() {
    if (_lastReading < 250) {
        _darkQuarterHours++;
    }



    // LED gets brighter the darker it is at the sensor
    // that means we have to -invert- the reading from 0-1023 back to 1023-0
    uint8_t l = 1023 - _lastReading;
    uint8_t brightness = map(l, 0, 1023, 0, 255);

    return (brightness);
}
