#include "Hygrometer.h"

Hygrometer::Hygrometer(void) {
    ADD_BIT(_status, STATUS_HYGR_GOOD);

    DEBUG_PRINTLN("Hygrometer loaded.");
}

void Hygrometer::begin(uint8_t pin) {
    _pin = pin;
    /* Set up the pins */
    pinMode(_pin, INPUT_PULLUP);

}

uint8_t Hygrometer::readHygrometer(void) {
	return 1;
}

uint8_t Hygrometer::processReading(void) {
	return 1;
}

uint8_t Hygrometer::getStatus(void) {
    return 1;
}
