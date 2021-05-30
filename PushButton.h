#ifndef PushButton_h
#define PushButton_h

#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
class PushButton
{
  public:
    PushButton();
    void setSounds(int soundMin, int soundMax);
    void setLed(uint8_t cathode, uint8_t anode, uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    void setValue(byte value);
    byte getValue();
    void setPin(int pin);
    void readPin();
    int getSound();
  private:
    uint8_t _cat;
    uint8_t _an;
    byte _value;
    int _soundMin;
    int _soundMax;
    uint16_t* _buffer;
    Adafruit_LEDBackpack* _matrix;
    void _ledOff();
    void _ledOn();
    void _dispMat();
    int _pin;
};

#endif
