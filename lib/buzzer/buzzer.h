#pragma once

#include <Arduino.h>

class BuzzerController {
private:
    uint8_t gpio_pin;
    bool is_playing;
    
public:
    BuzzerController(uint8_t pin = 10);
    void begin();
    void playTone(uint16_t frequency, bool enable);
    void turnOff();
    bool isPlaying() const { return is_playing; }
    uint8_t getPin() const { return gpio_pin; }
};