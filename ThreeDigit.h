#ifndef ThreeDigit_h
#define ThreeDigit_h

#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
class ThreeDigit
{
  public:
    ThreeDigit(uint8_t* cathodes, uint8_t* anodes, uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    void threeDigitDisp(int number);
    void init();
    int getValue();
  private:
    uint8_t* _cats;
    uint8_t* _ans;
    uint16_t* _buffer;
    int _value = 0;
    Adafruit_LEDBackpack* _matrix;
    void _digitDisp(uint8_t pos, uint8_t num);
    void _ledOff(uint8_t cat, uint8_t an);
    void _ledOn(uint8_t cat, uint8_t an);
    void _dispMat();
};

#endif
