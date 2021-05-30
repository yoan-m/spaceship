#include "Arduino.h"
#include "DoubleSwitch.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33

DoubleSwitch::DoubleSwitch()
{  
  _cat = -1;
  _an0 = 0;
  _an1 = 0;
  _an2 = 0;
  _sound0 = -1;
  _sound1 = -1;
  _sound2 = -1;
  _buffer = NULL;
  _matrix = NULL;
  _pin1 = -1;
  _pin2 = -1;
  _value = -1;
  _val1 = HIGH;
  _val2 = HIGH;
  _time = millis();
}



unsigned long DoubleSwitch::getTime(){
  return _time;
}
void DoubleSwitch::setSounds(int sound0, int sound1, int sound2){
  _sound0 = sound0;
  _sound1 = sound1;
  _sound2 = sound2;
}

void DoubleSwitch::setLed(uint8_t cathode, uint8_t anode0, uint8_t anode1, uint8_t anode2, uint16_t* buffer, Adafruit_LEDBackpack* matrix){
  
  _cat = cathode;
  _an0 = anode0;
  _an1 = anode1;
  _an2 = anode2;
  _buffer = buffer;
  _matrix = matrix;
}


void DoubleSwitch::setPins(int pin1, int pin2){
  _pin1 = pin1;
  _pin2 = pin2;
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
}
int DoubleSwitch::readPin(){
  _val1 = digitalRead(_pin1);
  _val2 = digitalRead(_pin2);
  
  if(_val1 == HIGH && _val2 == HIGH){
    setValue(0);
  }else if(_val1 == LOW){
    setValue(1);
  }else if(_val2 == LOW){
    setValue(2);
  }else{
    setValue(0);
  }
  return _value;
}

int DoubleSwitch::getSound(){
  if(_value == 2){
    Serial.println(_sound2);
    return _sound2;
  }else if(_value == 1){
    Serial.println(_sound1);
    return _sound1;
  }else {
    Serial.println(_sound0);
    return _sound0;
  }
}
int DoubleSwitch::getValue(){
  return _value;
}

void DoubleSwitch::setValue(int value)
{
  if(value == _value){
    return;
  }
  _value = value;
  _time = millis();
  if(_matrix != NULL){
      _led();
  }
  
  /*if(_value && _sound >= 0){
    _playSound();
  }*/
}


void DoubleSwitch::_led ()
{
  Serial.print("led ");
  Serial.println(_value);
  if(_value == 0){
    _buffer[_cat] |= _BV(_an0);
    if (_buffer[_cat] & _BV(_an1)) {
      _buffer[_cat] ^= _BV(_an1);
    }
    if (_buffer[_cat] & _BV(_an2)) {
      _buffer[_cat] ^= _BV(_an2);
    }
  } else if(_value == 1){
    _buffer[_cat] |= _BV(_an1);
    if (_buffer[_cat] & _BV(_an0)) {
      _buffer[_cat] ^= _BV(_an0);
    }
    if (_buffer[_cat] & _BV(_an2)) {
      _buffer[_cat] ^= _BV(_an2);
    }
  } else if(_value == 2){
  _buffer[_cat] |= _BV(_an2);
    if (_buffer[_cat] & _BV(_an0)) {
      _buffer[_cat] ^= _BV(_an0);
    }
    if (_buffer[_cat] & _BV(_an1)) {
      _buffer[_cat] ^= _BV(_an1);
    }
  } 
  _dispMat();
}


void DoubleSwitch::_playSound ()
{
  Serial.println("sound ");
}

void DoubleSwitch::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
