#include "Arduino.h"
#include "Led.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33


Led::Led(uint8_t cathode, uint8_t anode, uint16_t* buffer, Adafruit_LEDBackpack* matrix){
  
  _cat = cathode;
  _an = anode;
  _buffer = buffer;
  _matrix = matrix;
}

/*
void Led::setLed(uint8_t cathode, uint8_t anode, uint16_t* buffer, Adafruit_LEDBackpack* matrix){
  
  _cat = cathode;
  _an = anode;
  _buffer = buffer;
  _matrix = matrix;
}*/

void Led::off ()
{
  if (_buffer[_cat] & _BV(_an)) {
    _buffer[_cat] ^= _BV(_an);
  }
  _dispMat();
}

void Led::on ()
{
  _buffer[_cat] |= _BV(_an);
  _dispMat();
}

void Led::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
