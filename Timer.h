#ifndef Timer_h
#define Timer_h

#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33


const uint8_t MTdaysC[4] = {0,3,7,6};
const uint8_t MThoursC[2] = {4,1};
const uint8_t MTminutesC[2] = {2,5};
const uint8_t MTCat[8] = {6,0,3,7,4,1,2,5};
const uint8_t MTAn[8] = {7,5,6,2,0,4,1,3};
class Timer
{
  public:
    Timer(uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    void updateClock();
    void initClock (void);

  private:
    uint16_t* _buffer;
    
    uint8_t _days;
    uint8_t _minutes;
    uint8_t _hours;
    Adafruit_LEDBackpack* _matrix;
    void _digitDisp(uint8_t pos, uint8_t cathodes[], uint8_t anodes[], uint8_t num);
    void _ledOff(uint8_t cat, uint8_t an);
    void _ledOn(uint8_t cat, uint8_t an);
    void _dispMat();
};

#endif
