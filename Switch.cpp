#include "Arduino.h"
#include "Switch.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33

Switch::Switch()
{  
  _cat = 0;
  _an = 0;
  _soundOn = -1;
  _soundOff = -1;
  _buffer = NULL;
  _matrix = NULL;
  _pin = -1;
  _value = 0;
  _time = millis();
}




void Switch::setSounds(int soundOn, int soundOff){
  _soundOn = soundOn;
  _soundOff = soundOff;
}

void Switch::setLed(uint8_t cathode, uint8_t anode, uint16_t* buffer, Adafruit_LEDBackpack* matrix){
  
  _cat = cathode;
  _an = anode;
  _buffer = buffer;
  _matrix = matrix;
}


void Switch::setPin(int pin){
  _pin = pin;
  pinMode(pin, INPUT_PULLUP);
}
byte Switch::readPin(){
  if(_pin>0){
    setValue(digitalRead(_pin)== LOW ? 1 : 0);
  }
  return _value;
}


byte Switch::getValue(){
  return _value;
}

int Switch::getSound(){
  if(_value){
    Serial.println(_soundOn);
    return _soundOn;
  }else {
    Serial.println(_soundOff);
    return _soundOff;
  }
}
unsigned long Switch::getTime(){
  return _time;
}
void Switch::setValue(byte value)
{
  if(value == _value){
    return;
  }
  _value = value;
  _time = millis();
  if(_matrix != NULL){
    if(_value){
      _ledOn();
    }else{
      _ledOff();
    }
  }
}


void Switch::_ledOff ()
{
  Serial.println("ledOff ");
  if (_buffer[_cat] & _BV(_an)) {
    _buffer[_cat] ^= _BV(_an);
  }
  _dispMat();
}

void Switch::_ledOn ()
{
  Serial.println("ledOn ");
  _buffer[_cat] |= _BV(_an);
  _dispMat();
}


void Switch::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
