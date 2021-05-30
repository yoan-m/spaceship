#include "Arduino.h"
#include "Timer.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33


Timer::Timer(uint16_t buffer[], Adafruit_LEDBackpack* matrix)
{ 
  _buffer = buffer;
  _matrix = matrix;
  
  // Mission Timer
  _days = 0;
  _minutes = 0;
  _hours = 0;

}

void Timer::updateClock (void)
{

/*
      _digitDisp(0, MTdaysC, MTAn, 1);
      _digitDisp(1, MTdaysC, MTAn, 2);
      _digitDisp(2, MTdaysC, MTAn, 3);
    _digitDisp(0, MThoursC, MTAn, 4);
    _digitDisp(1, MThoursC, MTAn, 5);
  _digitDisp(0, MTminutesC, MTAn, 6);
  _digitDisp(1, MTminutesC, MTAn, 7);
  return;*/
  uint8_t ones, tens, hundreds, number;
  _minutes++;
  if (_minutes > 59) {
    _minutes = 0;
    _hours++;
    if (_hours > 24) {
      _hours = 0;
      _days++;
      if (_days > 999) {
        _days = 0;
      }
      hundreds = _days/100;
      number = _days-hundreds*100;
      tens = _days/10;
      ones = _days-tens*10;
      _digitDisp(0, MTdaysC, MTAn, hundreds);
      _digitDisp(1, MTdaysC, MTAn, tens);
      _digitDisp(2, MTdaysC, MTAn, ones);
    }
    //hundreds = hours/100;
    //number = hours-hundreds*100;
    tens = _hours/10;
    ones = _hours-tens*10;
    //digitDisp(1, MTdaysC, MTAn, hundreds);
    _digitDisp(0, MThoursC, MTAn, tens);
    _digitDisp(1, MThoursC, MTAn, ones);

  }
  tens = _minutes/10;
  ones = _minutes-tens*10;
  //digitDisp(1, MTdaysC, MTAn, hundreds);
  _digitDisp(0, MTminutesC, MTAn, tens);
  _digitDisp(1, MTminutesC, MTAn, ones);
}

void Timer::initClock (void)
{
  _digitDisp(3, MTCat, MTAn, 10);
  _digitDisp(5, MTCat, MTAn, 10);
  for (uint8_t i=0; i<8; i++) {
    _digitDisp(i, MTCat, MTAn, 0);
  }
}

void Timer::_digitDisp(uint8_t pos, uint8_t cat[], uint8_t an[], uint8_t num)
{
  switch (num) {
    case 11: // all off
    _ledOff(cat[(0+pos)], an[0]); // A
    _ledOff(cat[(0+pos)], an[1]); // B
    _ledOff(cat[(0+pos)], an[2]); // C
    _ledOff(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOff(cat[(0+pos)], an[5]); // F
    _ledOff(cat[(0+pos)], an[6]); // G
    break;

    case 10: // Decimal Point
    _ledOff(cat[(0+pos)], an[0]); // A
    _ledOff(cat[(0+pos)], an[1]); // B
    _ledOff(cat[(0+pos)], an[2]); // C
    _ledOff(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOff(cat[(0+pos)], an[5]); // F
    _ledOff(cat[(0+pos)], an[6]); // G
    break;

    case 0:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOn(cat[(0+pos)], an[3]); // D
    _ledOn(cat[(0+pos)], an[4]); // E
    _ledOn(cat[(0+pos)], an[5]); // F
    _ledOff(cat[(0+pos)], an[6]); // G
    break;

    case 1:
    _ledOff(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOff(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOff(cat[(0+pos)], an[5]); // F
    _ledOff(cat[(0+pos)], an[6]); // G
    break;
    
    case 2:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOff(cat[(0+pos)], an[2]); // C
    _ledOn(cat[(0+pos)], an[3]); // D
    _ledOn(cat[(0+pos)], an[4]); // E
    _ledOff(cat[(0+pos)], an[5]); // F
    _ledOn(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
    case 3:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOn(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOff(cat[(0+pos)], an[5]); // F
    _ledOn(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
    case 4:
    _ledOff(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOff(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOn(cat[(0+pos)], an[5]); // F
    _ledOn(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
    case 5:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOff(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOn(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOn(cat[(0+pos)], an[5]); // F
    _ledOn(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
    case 6:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOff(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOn(cat[(0+pos)], an[3]); // D
    _ledOn(cat[(0+pos)], an[4]); // E
    _ledOn(cat[(0+pos)], an[5]); // F
    _ledOn(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
    case 7:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOff(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOff(cat[(0+pos)], an[5]); // F
    _ledOff(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
    case 8:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOn(cat[(0+pos)], an[3]); // D
    _ledOn(cat[(0+pos)], an[4]); // E
    _ledOn(cat[(0+pos)], an[5]); // F
    _ledOn(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
    case 9:
    _ledOn(cat[(0+pos)], an[0]); // A
    _ledOn(cat[(0+pos)], an[1]); // B
    _ledOn(cat[(0+pos)], an[2]); // C
    _ledOn(cat[(0+pos)], an[3]); // D
    _ledOff(cat[(0+pos)], an[4]); // E
    _ledOn(cat[(0+pos)], an[5]); // F
    _ledOn(cat[(0+pos)], an[6]); // G
    //_ledOff(cat[(0+pos)], an[7]); // Decimal Point
    break;
    
  } // end switch(num)
} // end function digitDisp()

void Timer::_ledOff (uint8_t cat, uint8_t an)
{
  if (_buffer[cat] & _BV(an)) {
    _buffer[cat] ^= _BV(an);
  }
  _dispMat();
}

void Timer::_ledOn (uint8_t cat, uint8_t an)
{
  _buffer[cat] |= _BV(an);
  _dispMat();
}

void Timer::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
