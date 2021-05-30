#include "Arduino.h"
#include "PushButton.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
#include "DYPlayerArduino.h"


PushButton::PushButton()
{ 
}

void PushButton::setSounds(int soundMin, int soundMax){
  _soundMin = soundMin;
  _soundMax = soundMax;
}

void PushButton::setLed(uint8_t cathode, uint8_t anode, uint16_t* buffer, Adafruit_LEDBackpack* matrix){
  
  _cat = cathode;
  _an = anode;
  _buffer = buffer;
  _matrix = matrix;
}
void PushButton::setPin(int pin){
  _pin = pin;
  pinMode(pin, INPUT_PULLUP);
}
void PushButton::readPin(){
  if(_pin>0){
    setValue(digitalRead(_pin));
  }
}


byte PushButton::getValue(){
  return _value;
}

void PushButton::setValue(byte value)
{
  /*if(value == _value){
    return;
  }*/
  if(value == 1){
    _value = !_value;
  
    if(_matrix){
      if(_value){
        _ledOn();
      }else{
        _ledOff();
      }
    }
  }
}

int PushButton::getSound(){
  return random (_soundMin, _soundMax+1);
}

void PushButton::_ledOff ()
{
  if (_buffer[_cat] & _BV(_an)) {
    _buffer[_cat] ^= _BV(_an);
  }
  _dispMat();
}

void PushButton::_ledOn ()
{
  _buffer[_cat] |= _BV(_an);
  _dispMat();
}

void PushButton::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
