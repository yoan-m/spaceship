#ifndef Matrix_h
#define Matrix_h

#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
class Matrix
{
  public:
    Matrix(uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    //void setLed(uint8_t cathode, uint8_t anode, uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    void change();
  private:
    uint8_t _cat;
    uint8_t _an;
    uint16_t* _buffer;
    Adafruit_LEDBackpack* _matrix;
    void _ledOff(uint8_t cat, uint8_t an);
    void _ledOn(uint8_t cat, uint8_t an);
    void _dispMat();
};

#endif
