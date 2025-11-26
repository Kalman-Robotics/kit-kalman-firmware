#include "buzzer.h"

BuzzerController::BuzzerController(uint8_t pin) 
    : gpio_pin(pin), is_playing(false) {
}

void BuzzerController::begin() {
    pinMode(gpio_pin, OUTPUT);
    BuzzerController::turnOff();
}

void BuzzerController::playTone(uint16_t frequency, bool enable) {
    if (enable) {
        pinMode(gpio_pin, OUTPUT);
        tone(gpio_pin, frequency);
        is_playing = true;
    } else {
        BuzzerController::turnOff();
    }
}

void BuzzerController::turnOff() {
    noTone(gpio_pin);
    pinMode(gpio_pin, INPUT); // Deja el pin en alta impedancia 
    is_playing = false;
}