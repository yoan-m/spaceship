#ifndef Switch_h
#define Switch_h

#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
class Switch
{
  public:
    Switch();
    void setValue(byte value);
    void setSounds(int soundOn, int soundOff);
    void setLed(uint8_t cathode, uint8_t anode, uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    void setPin(int pin);
    byte readPin();
    byte getValue();
    int getSound();
    unsigned long getTime();
  private:
    uint8_t _cat;
    int _soundOn;
    int _soundOff;
    int _pin;
    unsigned long _time;
    uint8_t _an;
    byte _value;
    uint16_t* _buffer;
    Adafruit_LEDBackpack* _matrix;
    void _playSound();
    void _ledOff();
    void _ledOn();
    void _dispMat();
};

#endif
