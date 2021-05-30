#include "Matrix.h"
#include "Led.h"
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33



uint8_t ans[8] = {0,1,2,4,3,5,6,7};
uint8_t cats[8] = {7,3,5,1,0,2,6,4};
int cycle = 0;
uint8_t matrix1[64] = {0,0,0,1,1,0,0,0,0,0,1,1,1,1,0,0,0,1,1,1,1,1,1,0,1,1,0,1,1,0,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1};
uint8_t matrix2[64] = {
0,0,1,0,0,1,0,0,
0,0,1,0,0,1,0,0,
0,1,1,1,1,1,1,0,
1,1,0,1,1,0,1,1,
1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,
1,0,1,0,0,1,0,1,
0,0,1,0,0,1,0,0};
uint8_t matrix3[64] = {
0,0,1,0,0,1,0,0,
1,0,1,0,0,1,0,1,
1,1,1,1,1,1,1,1,
1,1,0,1,1,0,1,1,
1,1,1,1,1,1,1,1,
0,1,1,1,1,1,1,0,
0,0,1,0,0,1,0,0,
0,1,0,0,0,0,1,0};
uint8_t matrix4[64] = {
0,0,0,1,1,0,0,0,
0,0,1,1,1,1,0,0,
0,1,1,1,1,1,1,0,
1,1,0,1,1,0,1,1,
1,1,1,1,1,1,1,1,
0,0,1,0,0,1,0,0,
0,1,0,1,1,0,1,0,
0,1,0,0,0,0,1,0};
uint8_t matrix5[64] = {
0,0,0,1,1,0,0,0,
0,0,1,1,1,1,0,0,
0,1,1,1,1,1,1,0,
1,1,0,1,1,0,1,1,
1,1,1,1,1,1,1,1,
0,0,1,0,0,1,0,0,
0,1,0,1,1,0,1,0,
1,0,1,0,0,1,0,1};

uint8_t matrix6[64] = {
0,0,1,1,1,1,0,0,
0,1,1,1,1,1,1,0,
1,1,0,1,1,0,1,1,
1,1,0,1,1,0,1,1,
0,1,1,1,1,1,1,0,
0,0,1,0,0,1,0,0,
0,0,1,0,0,1,0,0,
0,0,1,0,0,1,0,0};
uint8_t matrix7[64] = {
0,0,0,0,0,0,0,0,
0,0,1,1,1,1,0,0,
0,1,1,1,1,1,1,0,
1,1,0,1,1,0,1,1,
1,1,0,1,1,0,1,1,
0,1,1,1,1,1,1,0,
0,0,1,0,0,1,0,0,
1,1,0,0,0,0,1,1};
Matrix::Matrix(uint16_t* buffer, Adafruit_LEDBackpack* matrix){
  
  _buffer = buffer;
  _matrix = matrix;
}

void Matrix::change(){
  uint8_t* matrix;
  switch(cycle){
    case 0:
    matrix = matrix2;
    cycle++;
    break;
    case 1:
    matrix = matrix3;
    cycle++;
    break;
    case 2:
    matrix = matrix2;
    cycle++;
    break;
    case 3:
    matrix = matrix2;
    cycle++;
    break;
    case 4:
    matrix = matrix3;
    cycle++;
    break;
    case 5:
    matrix = matrix2;
    cycle++;
    break;

    
    case 6:
    matrix = matrix4;
    cycle++;
    break;
    case 7:
    matrix = matrix5;
    cycle++;
    break;
    case 8:
    matrix = matrix4;
    cycle++;
    break;
    case 9:
    matrix = matrix5;
    cycle++;
    break;
    case 10:
    matrix = matrix4;
    cycle++;
    break;
    case 11:
    matrix = matrix5;
    cycle++;
    break;

  
    case 12:
    matrix = matrix6;
    cycle++;
    break;
    case 13:
    matrix = matrix7;
    cycle++;
    break;
    case 14:
    matrix = matrix6;
    cycle++;
    break;
    case 15:
    matrix = matrix7;
    cycle++;
    break;
    case 16:
    matrix = matrix6;
    cycle++;
    break;
    case 17:
    matrix = matrix7;
    cycle=0;
    break;
  
  }
  uint8_t i=0;
  for (uint8_t an=0; an<8; an++) {
    for (uint8_t cat=0; cat<8; cat++) {
      if(matrix[i]){
        _ledOn(cats[cat], ans[an]);
      }else{
        _ledOff(cats[cat], ans[an]);
      }
      i++;
    }
  }
}

void Matrix::_ledOff (uint8_t cat, uint8_t an)
{
  if (_buffer[cat] & _BV(an)) {
    _buffer[cat] ^= _BV(an);
  }
  _dispMat();
}

void Matrix::_ledOn (uint8_t cat, uint8_t an)
{
  _buffer[cat] |= _BV(an);
  _dispMat();
}

void Matrix::_dispMat()
{
  for (uint8_t i=0; i<8; i++) {
    _matrix->displaybuffer[i] = _buffer[i];
  }
  _matrix->writeDisplay();
}
