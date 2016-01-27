/******************************************************************************
* File: Photosensor.h                                        Garduino Library *
* Description: Header to handle a photosensor, detecting light amounts, for   *
*     use in the larger Garduino garden sensor project.                       *
*                                                                             *
* Copyright (c) 2016 At the Lamppost. All rights reserved.                    *
* Written by Trey Holdener <trey@atthelamppost.net>                           *
* MIT License (see LICENSE.txt)                                               *
******************************************************************************/

#ifndef __PHOTOSENSOR_H__

#define __PHOTOSENSOR_H__

#include "Garduino.h"

class Photosensor {
    public:
        Photosensor(void);
        void begin(uint8_t pin);
        uint8_t readPhotocell(void);
        uint8_t processReading(void);
        uint8_t getStatus(void);

    private:
        uint8_t _pin;
        uint8_t _darkQuarterHours;
        uint8_t _lastReading;
        uint8_t _status;
        bool _insufficientLight;
};

#endif
