#ifndef DoubleSwitch_h
#define DoubleSwitch_h

#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
class DoubleSwitch
{
  public:
    DoubleSwitch();
    void setValue(int value);
    void setSounds(int sound0, int sound1, int sound2);
    void setLed(uint8_t cathode, uint8_t anode0, uint8_t anode1, uint8_t anode2, uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    void setPins(int pin1, int pin2);
    int readPin();
    int getValue();
    int getSound();
    unsigned long getTime();
  private:
    uint8_t _cat;
    int _sound0;
    int _sound1;
    int _sound2;
    int _pin1;
    int _pin2;
    uint8_t _an0;
    uint8_t _an1;
    uint8_t _an2;
    unsigned long _time;
    int _value;
    int _val1;
    int _val2;
    uint16_t* _buffer;
    Adafruit_LEDBackpack* _matrix;
    void _playSound();
    void _led();
    void _dispMat();
};

#endif
