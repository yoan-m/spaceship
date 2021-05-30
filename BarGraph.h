#ifndef BarGraph_h
#define BarGraph_h

#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
  const uint8_t gten[] =    {1,1,1,1,1,1,1,1,1,1};
    const uint8_t gnine[] =   {0,1,1,1,1,1,1,1,1,1};
    const uint8_t geight[] =  {0,0,1,1,1,1,1,1,1,1};
    const uint8_t gseven[] =  {0,0,0,1,1,1,1,1,1,1};
    const uint8_t gsix[] =    {0,0,0,0,1,1,1,1,1,1};
    const uint8_t gfive[] =   {0,0,0,0,0,1,1,1,1,1};
    const uint8_t gfour[] =   {0,0,0,0,0,0,1,1,1,1};
    const uint8_t gthree[] =  {0,0,0,0,0,0,0,1,1,1};
    const uint8_t gtwo[] =    {0,0,0,0,0,0,0,0,1,1};
    const uint8_t gone[] =    {0,0,0,0,0,0,0,0,0,1};
    const uint8_t gzero[] =    {0,0,0,0,0,0,0,0,0,0};
class BarGraph
{
  public:
    BarGraph(int cathode, uint8_t* anodes, uint16_t* buffer, Adafruit_LEDBackpack* matrix);
    void potToGraph(int number);
    int getValue();
  private:
    
    uint8_t _cat;
    uint8_t* _ans;
    uint16_t* _buffer;
    Adafruit_LEDBackpack* _matrix;
    int _value = -1;
    void _barDisp();
    void _ledOff(uint8_t an);
    void _ledOn(uint8_t an);
    void _switchBars(uint8_t cmnd[]);
    void _dispMat();
};

#endif
