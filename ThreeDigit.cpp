#include "Arduino.h"
#include "ThreeDigit.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33


ThreeDigit::ThreeDigit(uint8_t cathodes[], uint8_t anodes[], uint16_t buffer[], Adafruit_LEDBackpack* matrix)
{  
  _cats = cathodes;
  _ans = anodes;
  _buffer = buffer;
  _matrix = matrix;
}

void ThreeDigit::init()
{
  threeDigitDisp(0);
}

int ThreeDigit::getValue(){
  return _value;
}

void ThreeDigit::threeDigitDisp(int number)
{
  _value = number;
  uint8_t ones, tens, hundreds;
  hundreds = number/100;
  number = number-hundreds*100;
  tens = number/10;
  ones = number-tens*10;
  _digitDisp(0, hundreds);
  _digitDisp(1, tens);
  _digitDisp(2, ones);
}

void ThreeDigit::_digitDisp(uint8_t pos, uint8_t num)
{
  switch (num) {
    case 11: // all off
    _ledOff(_cats[(0+pos)], _ans[0]); // A
    _ledOff(_cats[(0+pos)], _ans[1]); // B
    _ledOff(_cats[(0+pos)], _ans[2]); // C
    _ledOff(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOff(_cats[(0+pos)], _ans[5]); // F
    _ledOff(_cats[(0+pos)], _ans[6]); // G
    break;

    case 10: // Decimal Point
    _ledOff(_cats[(0+pos)], _ans[0]); // A
    _ledOff(_cats[(0+pos)], _ans[1]); // B
    _ledOff(_cats[(0+pos)], _ans[2]); // C
    _ledOff(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOff(_cats[(0+pos)], _ans[5]); // F
    _ledOff(_cats[(0+pos)], _ans[6]); // G
    break;

    case 0:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOn(_cats[(0+pos)], _ans[3]); // D
    _ledOn(_cats[(0+pos)], _ans[4]); // E
    _ledOn(_cats[(0+pos)], _ans[5]); // F
    _ledOff(_cats[(0+pos)], _ans[6]); // G
    break;

    case 1:
    _ledOff(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOff(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOff(_cats[(0+pos)], _ans[5]); // F
    _ledOff(_cats[(0+pos)], _ans[6]); // G
    break;
    
    case 2:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOff(_cats[(0+pos)], _ans[2]); // C
    _ledOn(_cats[(0+pos)], _ans[3]); // D
    _ledOn(_cats[(0+pos)], _ans[4]); // E
    _ledOff(_cats[(0+pos)], _ans[5]); // F
    _ledOn(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
    case 3:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOn(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOff(_cats[(0+pos)], _ans[5]); // F
    _ledOn(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
    case 4:
    _ledOff(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOff(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOn(_cats[(0+pos)], _ans[5]); // F
    _ledOn(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
    case 5:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOff(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOn(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOn(_cats[(0+pos)], _ans[5]); // F
    _ledOn(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
    case 6:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOff(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOn(_cats[(0+pos)], _ans[3]); // D
    _ledOn(_cats[(0+pos)], _ans[4]); // E
    _ledOn(_cats[(0+pos)], _ans[5]); // F
    _ledOn(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
    case 7:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOff(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOff(_cats[(0+pos)], _ans[5]); // F
    _ledOff(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
    case 8:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOn(_cats[(0+pos)], _ans[3]); // D
    _ledOn(_cats[(0+pos)], _ans[4]); // E
    _ledOn(_cats[(0+pos)], _ans[5]); // F
    _ledOn(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
    case 9:
    _ledOn(_cats[(0+pos)], _ans[0]); // A
    _ledOn(_cats[(0+pos)], _ans[1]); // B
    _ledOn(_cats[(0+pos)], _ans[2]); // C
    _ledOn(_cats[(0+pos)], _ans[3]); // D
    _ledOff(_cats[(0+pos)], _ans[4]); // E
    _ledOn(_cats[(0+pos)], _ans[5]); // F
    _ledOn(_cats[(0+pos)], _ans[6]); // G
    //_ledOff(_cats[(0+pos)], _ans[7]); // Decimal Point
    break;
    
  } // end switch(num)
} // end function digitDisp()

void ThreeDigit::_ledOff (uint8_t cat, uint8_t an)
{
  if (_buffer[cat] & _BV(an)) {
    _buffer[cat] ^= _BV(an);
  }
  _dispMat();
}

void ThreeDigit::_ledOn (uint8_t cat, uint8_t an)
{
  _buffer[cat] |= _BV(an);
  _dispMat();
}

void ThreeDigit::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
