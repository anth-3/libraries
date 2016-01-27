
#ifndef __HYGROMETER_H__

#define __HYGROMETER_H__

#include "Garduino.h"

class Hygrometer {
    public:
        Hygrometer(void);
        void begin(uint8_t pin);
        uint8_t readHygrometer(void);
        uint8_t processReading(void);
        uint8_t getStatus(void);

    private:
        uint8_t _pin;
        uint8_t _status;
        uint8_t _lastReading;

};

#endif
