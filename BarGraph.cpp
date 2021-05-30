#include "Arduino.h"
#include "BarGraph.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33


BarGraph::BarGraph(int cathode, uint8_t anodes[], uint16_t buffer[], Adafruit_LEDBackpack* matrix)
{  
  _cat = cathode;
  _ans = anodes;
  _buffer = buffer;
  _matrix = matrix;
}


int BarGraph::getValue(){
  return _value;
}


void BarGraph::potToGraph (int value)
{
    if (_value == -1 || abs(value - _value) > 0) {
      _value = value;
      #ifdef DEBUG
        Serial.print("potToGraph ");
        Serial.print(_value);
        Serial.println();
      #endif
      _barDisp();
  }
}

void BarGraph::_barDisp ()
{
  switch (_value) {
    case 10:
    _switchBars(gten);
    break;

    case 9:
    _switchBars(gnine);
    break;

    case 8:
    _switchBars(geight);
    break;

    case 7:
    _switchBars(gseven);
    break;

    case 6:
    _switchBars(gsix);
    break;

    case 5:
    _switchBars(gfive);
    break;

    case 4:
    _switchBars(gfour);
    break;

    case 3:
    _switchBars(gthree);
    break;

    case 2:
    _switchBars(gtwo);
    break;

    case 1:
    _switchBars(gone);
    break;
    
    case 0:
    _switchBars(gzero);
    break;
  }
}  


void BarGraph::_switchBars (uint8_t cmnd[])
{
  #ifdef DEBUG
    Serial.print("switchBars ");
    Serial.println();
  #endif
  for (uint8_t i=0; i<10; i++)
  {
    if (cmnd[i] == 1) {
      _ledOn(_ans[i]);
    } else {
      _ledOff(_ans[i]);
    }
  }
}

void BarGraph::_ledOff (uint8_t an)
{
  if (_buffer[_cat] & _BV(an)) {
    _buffer[_cat] ^= _BV(an);
  }
  _dispMat();
}

void BarGraph::_ledOn (uint8_t an)
{
  _buffer[_cat] |= _BV(an);
  _dispMat();
}

void BarGraph::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
