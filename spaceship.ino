#include <Wire.h> // For i2c communications with the MCP23017 i/o expanders and the HT16K33 LED matrix driver
#include "Adafruit_LEDBackpack.h" // For the HT16K33
#include "Adafruit_GFX.h" // For the HT16K33
#include "DYPlayerArduino.h"
#include "ThreeDigit.h"
#include "BarGraph.h"
#include "Timer.h"
#include "DoubleSwitch.h"
#include "Switch.h"
#include "PushButton.h"
#include "Led.h"
#include "Matrix.h"
#include <FastLED.h>
FASTLED_USING_NAMESPACE
uint8_t stateLed[16];

#define FRAMES_PER_SECOND  120
#define DATA_PIN    3
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    76
CRGB leds[NUM_LEDS];
uint8_t gCurrentPatternNumber = 0;
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
int brightness = -1;

DY::Player player;
int volume = -1;


#define DEBUG 1
#define I2C_SLAVE1_ADDRESS 4
Adafruit_LEDBackpack matrixA = Adafruit_LEDBackpack(); //NONE
Adafruit_LEDBackpack matrixB = Adafruit_LEDBackpack(); //BARGRAPH
Adafruit_LEDBackpack matrixC = Adafruit_LEDBackpack(); //AHR ABR
Adafruit_LEDBackpack matrixD = Adafruit_LEDBackpack();//ATTI YAW ROLL PITCH IHR
Adafruit_LEDBackpack matrixE = Adafruit_LEDBackpack();
Adafruit_LEDBackpack matrixF = Adafruit_LEDBackpack();
Adafruit_LEDBackpack matrixG = Adafruit_LEDBackpack();

#define potRef 950

#define matAaddy 0x70
#define matBaddy 0x74
#define matCaddy 0x72
#define matDaddy 0x73
#define matEaddy 0x71
#define matFaddy 0x75
#define matGaddy 0x76

// MCP23017 registers (everything except direction defaults to 0)
#define IODIRA   0x00   // IO direction  (0 = output, 1 = input (Default))
#define IODIRB   0x01
#define IOPOLA   0x02   // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB   0x03
#define GPINTENA 0x04   // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB 0x05
#define DEFVALA  0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB  0x07
#define INTCONA  0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define INTCONB  0x09
#define IOCON    0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
//#define IOCON 0x0B  // same as 0x0A
#define GPPUA    0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define GPPUB    0x0D
#define INFTFA   0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INFTFB   0x0F
#define INTCAPA  0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define INTCAPB  0x11
#define GPIOA    0x12   // Port value. Write to change, read to obtain value
#define GPIOB    0x13
#define OLLATA   0x14   // Output latch. Write to latch output.
#define OLLATB   0x15

#define expA 0x20
#define expB 0x23
#define expC 0x22
#define expD 0x24

uint8_t state[64]; // buffer for state of buttons
uint16_t ABuffer[8]; // buffer for LED matrix A
uint16_t BBuffer[8]; // buffer for LED matrix B
uint16_t CBuffer[8]; // buffer for LED matrix C
uint16_t DBuffer[8]; // buffer for LED matrix D
uint16_t EBuffer[8]; // buffer for LED matrix E
uint16_t FBuffer[8]; // buffer for LED matrix F
uint16_t GBuffer[8]; // buffer for LED matrix G

uint8_t pitchCat[4] = {0,1,2};
uint8_t pitchAn[8] = {0,1,2,3,4,5,6};
uint8_t yawCat[4] = {3,4,5};
uint8_t yawAn[8] = {0,1,2,3,4,5,6};
uint8_t rollCat[4] = {0,1,2};
uint8_t rollAn[8] = {8,9,10,11,12,13,14};
uint8_t ihrCat[4] = {3,4,5};
uint8_t ihrAn[8] = {8,9,10,11,12,13,14};

uint8_t abrCat[4] = {0,1,2};
uint8_t abrAn[8] = {8,9,10,11,12,13,14};
uint8_t ahrCat[4] = {3,4,5};
uint8_t ahrAn[8] = {8,9,10,11,12,13,14};
uint8_t cryoO2PCat[4] = {0,1,2};
uint8_t cryoO2PAn[8] = {0,1,2,3,4,5,6};
uint8_t cryoH2PCat[4] = {3,4,5};
uint8_t cryoH2PAn[8] = {0,1,2,3,4,5,6};


uint8_t statCat[8] = {0,1,2,3,4,5,6,7};
uint8_t statAn[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
uint8_t grphAns[10] = {0,1,2,3,4,5,6,7,8,9};
boolean reset = false;

ThreeDigit yaw(yawCat, yawAn, DBuffer, &matrixD);
ThreeDigit roll(rollCat, rollAn, DBuffer, &matrixD);
ThreeDigit pitch(pitchCat, pitchAn, DBuffer, &matrixD);
ThreeDigit ihr(ihrCat, ihrAn, DBuffer,&matrixD);
ThreeDigit abr(abrCat, abrAn, CBuffer, &matrixC);
ThreeDigit ahr(ahrCat, ahrAn, CBuffer, &matrixC);
ThreeDigit cryoO2P(cryoO2PCat, cryoO2PAn, FBuffer, &matrixF);
ThreeDigit cryoH2P(cryoH2PCat, cryoH2PAn, FBuffer, &matrixF);

Timer timer(ABuffer, &matrixA);

Matrix matrix(GBuffer, &matrixG);

//CONTROL
Switch control_docking_probe;//Docking probe
Switch control_glycol_pump;//Glycol pump
Switch control_sce_power;//SCE power
Switch control_waste_dump;//Waste dump
Switch control_cabin_fan;//Cabin fan
Switch control_h2o_flow;//H2O flow
Switch control_int_lights;//Int lights
Switch control_suit_comp;//Suit comp*


//MISSION
PushButton mission_01;//Mission 1
PushButton mission_02;//Mission 2
PushButton mission_03;//Mission 3
PushButton mission_04;//Mission 4
PushButton mission_05;//Mission 5
PushButton mission_06;//Mission 6
PushButton mission_07;//Mission 7
PushButton mission_08;//Mission 8
PushButton mission_09;//Mission 9
PushButton mission_10;//Mission 10

//C&WS
Switch cws_lamp;//LAMP
PushButton cws_push;
Switch cws_ack;//ACK
Switch cws_power;//POWER
Switch cws_mode;//MODE

//ABORT
PushButton abort_push;
Switch abort_arm;//ARM
Switch abort_mode_pad;//MODE
Switch abort_mode_1;//MODE
Switch abort_mode_2;//MODE
Switch abort_mode_3;//MODE
Switch abort_mode_4;//MODE
Switch abort_mode_5;//MODE

//INCO
Switch inco_radio;

//CAPCOM
Switch capcom;
Switch capcom_key;

//BOOSTER
Switch booster_sps;//SPS
Switch booster_tei;//TEI
Switch booster_tli;//TLI
Switch booster_sic;//SIC
Switch booster_sii;//SII
Switch booster_sivb;//SIVB
Switch booster_mi;//MI
Switch booster_mii;//MII
Switch booster_miii;//MIII


//PYRITECHNICS
Switch pyro_main_deploy;//MAIN DEPLOY
Switch pyro_drogue_deploy;//DROGUE DEPLOY
Switch pyro_csmlv_sep;//CSMLV SEP
Switch pyro_canard_deploy;//CANARD DEPLOY
Switch pyro_smcm_sep;//SMCM SEP
Switch pyro_apex_cover_jet;//APEX COVER JET
Switch pyro_less_motor_fire;//LES MOTOR FIRE

//CRYOGENICS
DoubleSwitch cryo_o2_fan;
DoubleSwitch cryo_pumps;
DoubleSwitch cryo_h2_fan;
DoubleSwitch cryo_heat;
uint8_t grphCryoAns[10] = {9,8,7,6,5,4,3,2,1,0};
BarGraph barO2(7,grphCryoAns,EBuffer,&matrixE);
BarGraph barH2(6,grphCryoAns,EBuffer,&matrixE);
BarGraph cryoO2Q(5,grphAns,BBuffer,&matrixB);
BarGraph cryoH2Q(6,grphAns,BBuffer,&matrixB);

//EECOM
BarGraph barVolt(3,grphAns,BBuffer,&matrixB);
BarGraph barOhm(2,grphAns,BBuffer,&matrixB);
BarGraph barCur(1,grphAns,BBuffer,&matrixB);
BarGraph barOx(0,grphAns,BBuffer,&matrixB);
BarGraph barSlider(4,grphAns,BBuffer,&matrixB);

//ALARMS
Led alarmBMag1Temp(0,0,EBuffer,&matrixE);
Led alarmPitchGmbl1(0,1,EBuffer,&matrixE);
Led alarmPitchGmbl2(0,2,EBuffer,&matrixE);
Led alarmGlycolTempLow(0,3,EBuffer,&matrixE);
Led alarmSMRCSA(0,4,EBuffer,&matrixE);
Led alarmSMRCSD(0,5,EBuffer,&matrixE);
Led alarmBMag2Temp(1,0,EBuffer,&matrixE);
Led alarmYawGmbl1(1,1,EBuffer,&matrixE);
Led alarmYawGmbl2(1,2,EBuffer,&matrixE);
Led alarmCMRCS1(1,3,EBuffer,&matrixE);
Led alarmSMRCSB(1,4,EBuffer,&matrixE);
Led alarmUplinkActivity(1,5,EBuffer,&matrixE);
Led alarmCO2PPHI(2,0,EBuffer,&matrixE);
Led alarmHgAntScanLimit(2,1,EBuffer,&matrixE);
Led alarmCryoPress(2,2,EBuffer,&matrixE);
Led alarmCMRCS2(2,3,EBuffer,&matrixE);
Led alarmSMRCSC(2,4,EBuffer,&matrixE);
Led alarmGimbalLock(2,5,EBuffer,&matrixE);
Led alarmDrogueChute(3,0,EBuffer,&matrixE);
Led alarmMainChute(3,1,EBuffer,&matrixE);
Led alarmSPSPress(3,2,EBuffer,&matrixE);
Led alarmSPSRoughEco(3,3,EBuffer,&matrixE);
Led alarmSPSFlngTempHi(3,4,EBuffer,&matrixE);
Led alarmFCBusDicnnt(3,5,EBuffer,&matrixE);
Led alarmUlage(4,0,EBuffer,&matrixE);
Led alarmMatch(4,1,EBuffer,&matrixE);
Led alarmACBus1(4,2,EBuffer,&matrixE);
Led alarmMnBusAUndervolt(4,3,EBuffer,&matrixE);
Led alarmCrewAlert(4,4,EBuffer,&matrixE);
Led alarmO2FlowHi(4,5,EBuffer,&matrixE);
Led alarmThrust(5,0,EBuffer,&matrixE);
Led alarmDockingTarget(5,1,EBuffer,&matrixE);
Led alarmAcBus2(5,2,EBuffer,&matrixE);
Led alarmMnBusUndervolt(5,3,EBuffer,&matrixE);
Led alarmCW(5,4,EBuffer,&matrixE);
Led alarmSuitComp(5,5,EBuffer,&matrixE);


unsigned long numericTimer = 0;
#define NUMERICREFRESH 2000
unsigned long cryoTimer = 0;
#define TIMEOUT 10000
uint8_t numericPos = 0;
unsigned long clockTimer = 0;



void allMatrixOff(void)
{
  matrixOff(matrixA, ABuffer);
  matrixOff(matrixB, BBuffer);
  matrixOff(matrixC, CBuffer);
  matrixOff(matrixD, DBuffer);
  matrixOff(matrixE, EBuffer);
}

void matrixOff(Adafruit_LEDBackpack matrix, uint16_t buffer[8])
{
  for (uint8_t i=0; i<8; i++) {
    buffer[i] = 0;
  }
  dispMat(matrix, buffer);
}

void ledOn (Adafruit_LEDBackpack matrix, uint16_t buffer[8], uint8_t cat, uint8_t an)
{
  buffer[cat] |= _BV(an);
  dispMat(matrix, buffer);
}

void ledOff (Adafruit_LEDBackpack matrix, uint16_t buffer[8], uint8_t cat, uint8_t an)
{
  if (buffer[cat] & _BV(an)) {
    buffer[cat] ^= _BV(an);
  }
  dispMat(matrix, buffer);
}

void dispMat (Adafruit_LEDBackpack matrix, uint16_t buffer[8])
{
  for (uint8_t i=0; i<8; i++) {
    matrix.displaybuffer[i] = buffer[i];
  }
  matrix.writeDisplay();
}

void lampTestOn (void)
{
  /*for (uint8_t cathode=0; cathode<6; cathode++) {
    for (uint8_t anode=0; anode<12; anode++) {
      ledOn(matrixE, EBuffer, cathode, anode);
    }
  }/*
  for (uint8_t cathode=0; cathode<8; cathode++) {
    for (uint8_t anode=0; anode<12; anode++) {
      ledOn(matrixF, FBuffer, cathode, anode);
    }
  }*/
  for (uint8_t cathode=0; cathode<8; cathode++) {
    for (uint8_t anode=0; anode<12; anode++) {
      ledOn(matrixG, GBuffer, cathode, anode);
    }
  }
  //ledOn(matrixE, EBuffer, 2, 7);
  //ledOn(matrixE, EBuffer, 4, 4);
}

void lampTestOff (void)
{
  for (uint8_t cathode=0; cathode<6; cathode++) {
    for (uint8_t anode=0; anode<6; anode++) {
      ledOff(matrixE, EBuffer, statCat[cathode], statAn[anode]);
    }
  }
  ledOff(matrixE, EBuffer, 2, 7);
  ledOff(matrixE, EBuffer, 4, 4);
}

void expanderWritePort(const byte reg, const byte data, const byte theport)
{
  Wire.beginTransmission (theport);
  Wire.write (reg);
  Wire.write (data);  // port A
  Wire.endTransmission ();
}

void expanderWriteBothPort(const byte reg, const byte data, const byte theport)
{
  Wire.beginTransmission (theport);
  Wire.write (reg);
  Wire.write (data);  // port A
  Wire.write (data);  // port B
  Wire.endTransmission ();
} // end of expanderWriteBothPort

// read a byte from the expander
unsigned int expanderReadPort (const byte reg, int theport) 
{
  Wire.beginTransmission (theport);
  Wire.write (reg);
  Wire.endTransmission ();
  Wire.requestFrom (theport, 1);
  int ret;
  while(Wire.available())    // slave may send less than requested
  {
    ret = Wire.read();    // receive a byte as character
    //Serial.print(c);         // print the character
  }

  delay(4);
  return ret;
  //return Wire.read();
} // end of expanderRead



#define CONTROL_WASTE_PUMP 0
#define CONTROL_SCE_POWER 1
#define CONTROL_GLYCOL_PUMP 2
#define CONTROL_DOCKING_PROBE 3
#define CONTROL_CABIN_FAN 7
#define CONTROL_H2O_FLOW 6
#define CONTROL_INT_LIGHTS 5
#define CONTROL_SUIT_COMP 4

#define MISSION_01 0
#define MISSION_02 9
#define MISSION_03 11
#define MISSION_04 13
#define MISSION_05 15
#define MISSION_06 0
#define MISSION_07 9
#define MISSION_08 10
#define MISSION_09 12
#define MISSION_10 14
void scanButtons(void) {
  byte current[64];
  uint16_t value = 0;  
  value |= expanderReadPort(GPIOA, expA) << 8;
  value |= expanderReadPort(GPIOB, expA);
  for (uint8_t i=0; i<16; i++) {
    if (value & (1 << i)) {
      current[i] = 1;
    } else {
      current[i] = 0;
    }
  }
  value = 0;
  value |= expanderReadPort(GPIOA, expB) << 8;
  value |= expanderReadPort(GPIOB, expB);
  for (uint8_t i=0; i<16; i++) {
    if (value & (1 << i)) {
      current[i+16] = 1;
    } else {
      current[i+16] = 0;
    }
  }
  value = 0;
  value |= expanderReadPort(GPIOA, expC) << 8;
  value |= expanderReadPort(GPIOB, expC);
  for (uint8_t i=0; i<16; i++) {
    if (value & (1 << i)) {
      current[i+32] = 1;
    } else {
      current[i+32] = 0;
    }
  }
  value = 0;
  value |= expanderReadPort(GPIOA, expD) << 8;
  value |= expanderReadPort(GPIOB, expD);
  for (uint8_t i=0; i<16; i++) {
    if (value & (1 << i)) {
      current[i+48] = 1;
    } else {
      current[i+48] = 0;
    }
  }
  for (uint8_t i=0; i<64; i++) {
    if (((current[i] == 0) && (state[i] == 1)) ||
    ((current[i] == 1) && (state[i] == 0))) { // Switch was closed, now open      
      #ifdef DEBUG
        if(current[i] == 0){
          Serial.print("open");
        }else{
          Serial.print("close");
        }
        Serial.println(i);
      #endif
      state[i] = current[i];
      //if (i==17) lampTestOff();
      switch(i){
        case 0:
        mission_08.setValue(current[i]);
        if(mission_08.getValue()){
          sendLight(106);
          player.playSpecified(mission_08.getSound());
        }
        break;
        case 1:
        mission_10.setValue(current[i]);
        if(mission_10.getValue()){
          sendLight(108);
          player.playSpecified(mission_10.getSound());
        }
        break;
        case 2:
        mission_03.setValue(current[i]);
        if(mission_03.getValue()){
          sendLight(100);
          player.playSpecified(mission_03.getSound());
        }
        break;
        case 3:
        mission_02.setValue(current[i]);
        if(mission_02.getValue()){
          sendLight(103);
          player.playSpecified(mission_02.getSound());
        }
        break;
        case 4:
        mission_07.setValue(current[i]);
        if(mission_07.getValue()){
          sendLight(105);
          player.playSpecified(mission_07.getSound());
        }
        break;
        case 5:
        mission_05.setValue(current[i]);
        if(mission_05.getValue()){
          sendLight(102);
          player.playSpecified(mission_05.getSound());
        }
        break;
        case 6:
        mission_09.setValue(current[i]);
        if(mission_09.getValue()){
          sendLight(107);
          player.playSpecified(mission_09.getSound());
        }
        break;
        case 7:
        mission_04.setValue(current[i]);
        if(mission_04.getValue()){
          sendLight(101);
          player.playSpecified(mission_04.getSound());
        }
        break;
        case 9:
        cws_lamp.setValue(current[i]);
        if(cws_lamp.getValue()){
          player.playSpecified(cws_lamp.getSound());
        }
        break;
        case 10:
        cws_push.setValue(current[i]);
        if(cws_push.getValue()){
          player.playSpecified(cws_push.getSound());
        }
        break;
        case 11:
        cws_ack.setValue(current[i]);
        if(cws_ack.getValue()){
          player.playSpecified(cws_ack.getSound());
        }
        break;
        case 12:
        cws_power.setValue(current[i]);
        if(cws_power.getValue()){
          player.playSpecified(cws_power.getSound());
        }
        break;
        case 13:
        cws_mode.setValue(current[i]);
        if(cws_mode.getValue()){
          player.playSpecified(cws_mode.getSound());
        }
        break;
        case 14:
        mission_01.setValue(current[i]);
        if(mission_01.getValue()){
          player.playSpecified(mission_01.getSound());
        }
        break;
        case 15:
        mission_06.setValue(current[i]);
        if(mission_06.getValue()){
          sendLight(104);
          player.playSpecified(mission_06.getSound());
        }
        break;
        case 8:
        abort_push.setValue(current[i]);
        if(abort_push.getValue()){
          sendLight(109);
          player.playSpecified(abort_push.getSound());
        }
        break;
        case 31:
        /*capcom.setValue(current[i]);
        if(capcom.getValue()){
          player.playSpecified(capcom.getSound());
        }*/
        break;
        case 33:
        pyro_main_deploy.setValue(current[i]);
        if(pyro_main_deploy.getValue()){
          player.playSpecified(pyro_main_deploy.getSound());
        }
        break;
        case 34:
        pyro_drogue_deploy.setValue(current[i]);
        if(pyro_drogue_deploy.getValue()){
          player.playSpecified(pyro_drogue_deploy.getSound());
        }
        break;
        case 35:
        pyro_csmlv_sep.setValue(current[i]);
        if(pyro_csmlv_sep.getValue()){
          player.playSpecified(pyro_csmlv_sep.getSound());
        }
        break;
        case 36:
        pyro_canard_deploy.setValue(current[i]);
        if(pyro_canard_deploy.getValue()){
          player.playSpecified(pyro_canard_deploy.getSound());
        }
        break;
        case 37:
        pyro_smcm_sep.setValue(current[i]);
        if(pyro_smcm_sep.getValue()){
          player.playSpecified(pyro_smcm_sep.getSound());
        }
        break;
        case 38:
        pyro_apex_cover_jet.setValue(current[i]);
        if(pyro_apex_cover_jet.getValue()){
          player.playSpecified(pyro_apex_cover_jet.getSound());
        }
        break;
        case 39:
        pyro_less_motor_fire.setValue(current[i]);
        if(pyro_less_motor_fire.getValue()){
          player.playSpecified(pyro_less_motor_fire.getSound());
        }
        break;
        case 40:
        booster_sps.setValue(current[i]);
        if(booster_sps.getValue()){
          player.playSpecified(booster_sps.getSound());
        }
        break;
        case 41:
        booster_tei.setValue(current[i]);
        if(booster_tei.getValue()){
          player.playSpecified(booster_tei.getSound());
        }
        break;
        case 42:
        booster_tli.setValue(current[i]);
        if(booster_tli.getValue()){
          player.playSpecified(booster_tli.getSound());
        }
        break;
        case 43:
        booster_sic.setValue(current[i]);
        if(booster_sic.getValue()){
          player.playSpecified(booster_sic.getSound());
        }
        break;
        case 44:
        booster_sii.setValue(current[i]);
        if(booster_sii.getValue()){
          player.playSpecified(booster_sii.getSound());
        }
        break;
        case 45:
        booster_sivb.setValue(current[i]);
        if(booster_sivb.getValue()){
          player.playSpecified(booster_sivb.getSound());
        }
        break;
        case 46:
        booster_mi.setValue(current[i]);
        if(booster_mi.getValue()){
          player.playSpecified(booster_mi.getSound());
        }
        break;
        case 47:
        booster_mii.setValue(current[i]);
        if(booster_mii.getValue()){
          player.playSpecified(booster_mii.getSound());
        }
        break;
        case 48:
        booster_miii.setValue(current[i]);
        if(booster_miii.getValue()){
          player.playSpecified(booster_miii.getSound());
        }
        break;
      }
    }
  }  
  byte val1;
  byte val2;
  
  val2 = capcom.getValue();
  val1 = capcom.readPin();
  if(val2 != val1){
    player.playSpecified(capcom.getSound());
  }
  val2 = control_docking_probe.getValue();
  val1 = control_docking_probe.readPin();
  if(val2 != val1){
    //if(val1){
        player.playSpecified(control_docking_probe.getSound());
    //}
  }
  val2 = control_glycol_pump.getValue();
  val1 = control_glycol_pump.readPin();
  if(val2 != val1){
    //if(val1){
        player.playSpecified(control_glycol_pump.getSound());
    //}
  }
  val2 = control_sce_power.getValue();
  val1 = control_sce_power.readPin();
  if(val2 != val1){
    //if(val1){
        player.playSpecified(control_sce_power.getSound());
    //}
  }
  val2 = control_waste_dump.getValue();
  val1 = control_waste_dump.readPin();
  if(val2 != val1){
    //if(val1){
        player.playSpecified(control_waste_dump.getSound());
    //}
  }
  val2 = control_cabin_fan.getValue();
  val1 = control_cabin_fan.readPin();
  if(val2 != val1){
    //if(val1){
        player.playSpecified(control_cabin_fan.getSound());
    //}
  }
  val2 = control_h2o_flow.getValue();
  val1 = control_h2o_flow.readPin();
  if(val2 != val1){
    //if(val1){
        player.playSpecified(control_h2o_flow.getSound());
    //}
  }
  val2 = control_int_lights.getValue();
  val1 = control_int_lights.readPin();
  if(val2 != val1){
    if(val1){
      sendLight(10);
    }else{
      sendLight(-1);
    }
    player.playSpecified(control_int_lights.getSound());
  }
  val2 = control_suit_comp.getValue();
  val1 = control_suit_comp.readPin();
  if(val2 != val1){
    //if(val1){
        player.playSpecified(control_suit_comp.getSound());
    //}
  }
  val2 = abort_arm.getValue();
  val1 = abort_arm.readPin();
  if(val2 != val1){
    if(val1){
        player.playSpecified(abort_arm.getSound());
    }
  }
  abort_mode_pad.readPin();
  abort_mode_1.readPin();
  abort_mode_2.readPin();
  abort_mode_3.readPin();
  abort_mode_4.readPin();
  abort_mode_5.readPin();
 
  val2 = inco_radio.getValue();
  val1 = inco_radio.readPin();
  if(val2 != val1){
    if(val1){
        player.playSpecified(inco_radio.getSound());
    }
  }
  val2 = cryo_o2_fan.getValue();
  val1 = cryo_o2_fan.readPin();
  if(val2 != val1){
    if(val1){
        player.playSpecified(cryo_o2_fan.getSound());
    }
  }
  val2 = cryo_pumps.getValue();
  val1 = cryo_pumps.readPin();
  if(val2 != val1){
    if(val1){
        player.playSpecified(cryo_pumps.getSound());
    }
  }
  val2 = cryo_h2_fan.getValue();
  val1 = cryo_h2_fan.readPin();
  if(val2 != val1){
    if(val1){
        player.playSpecified(cryo_h2_fan.getSound());
    }
  }
  val2 = cryo_heat.getValue();
  val1 = cryo_heat.readPin();
  if(val2 != val1){
    if(val1){
        player.playSpecified(cryo_heat.getSound());
    }
  }
}



void sendLight(int value){

  //10 white
  if(abort_arm.getValue()){
    if(abort_mode_1.getValue()){
      gCurrentPatternNumber = 2;
      confetti();
    }else if(abort_mode_2.getValue()){
      gCurrentPatternNumber = 3;
      sinelon();
    }else if(abort_mode_3.getValue()){
      gCurrentPatternNumber = 4;
      juggle();
    }else if(abort_mode_4.getValue()){
      gCurrentPatternNumber = 5;
      bpm();
    }else if(abort_mode_5.getValue()){
      gCurrentPatternNumber = 6;
      rainbow();
    }else{
      gCurrentPatternNumber = 1;
      rainbowWithGlitter();
    } 
  }else if(control_int_lights.getValue()){
   if(gCurrentPatternNumber != 10){
      for (int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
        leds[whiteLed] = CRGB::White;
      }
      gCurrentPatternNumber = 10;
    }
  }
  else if(value == -1){
    gCurrentPatternNumber = 0;
    FastLED.clear();
  }else{
    fadeToBlackBy(leds, NUM_LEDS, 20);
    if(value == gCurrentPatternNumber){
      gCurrentPatternNumber = 0;
      return;
    }
    gCurrentPatternNumber = value;
    if(!value){
      return;
    }
    CRGB color = CRGB::White;
    switch (value) {
      case 100:
        color = CRGB::Red;
        break;
      case 101:
        color = CRGB::Yellow;
        break;
      case 102:
        color = CRGB::Green;
        break;
      case 103:
        color = CRGB::Blue;
        break;
      case 104:
        color = CRGB::Pink;
        break;
      case 105:
        color = CRGB::OrangeRed;
        break;
      case 106:
        color = CRGB::White;
        break;
      case 107:
        color = CRGB::OliveDrab;
        break;
      case 108:
        color = CRGB::MediumAquamarine;
        break;
      case 109:
        color = CRGB::Purple;
        break;
    }
    // Move a single white led
    for (int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
      leds[whiteLed] = color;
    }
  }
}


void computeCryo (void){
  int valO2 = cryoO2P.getValue();
  int pumps = cryo_pumps.getValue();
  if(cryo_o2_fan.getValue() == 0){
    if(valO2 >= 0 ){
      valO2-=random(1,3);
    }
  }else if(cryo_o2_fan.getValue() == 1){
    if(valO2 > 150 ){
      valO2-=random(1,3);
    }else{
      if(pumps == 1){
        valO2+=random(1,2);
      }else if(pumps == 1){
        valO2+=random(2,3);
      }else{
        valO2++;
      }
    }
  }else{
    if(valO2 <= 300 ){
      if(pumps == 1){
        valO2+=random(1,2);
      }else if(pumps == 1){
        valO2+=random(2,3);
      }else{
        valO2++;
      }
    }
  }
  cryoO2P.threeDigitDisp(valO2);
  barO2.potToGraph(round(valO2/30));

  
  int valH2 = cryoH2P.getValue();
  if(cryo_h2_fan.getValue() == 0){
    if(valH2 >= 0 ){
      valH2-=random(1,3);
    }
  }else if(cryo_h2_fan.getValue() == 1){
    if(valH2 > 150 ){
      valH2-=random(1,3);
    }else{
      if(pumps == 1){
        valH2+=random(1,2);
      }else if(pumps == 1){
        valH2+=random(2,3);
      }else{
        valH2++;
      }
    }
  }else{
    if(valH2 <= 300 ){
      if(pumps == 1){
        valH2+=random(1,2);
      }else if(pumps == 1){
        valH2+=random(2,3);
      }else{
        valH2++;
      }
    }
  }
  cryoH2P.threeDigitDisp(valH2);
  barH2.potToGraph(round(valH2/30));
}

void alarms(){
  unsigned long timer = millis();
  bool alert = false;


  if(!pyro_main_deploy.getValue()){
    alarmMainChute.on();
  }else{
    alarmMainChute.off();
  }

  alarmUlage.on();
  alarmMatch.on();

  if(!pyro_less_motor_fire.getValue()){
    alarmThrust.on();
  }else{
    alarmThrust.off();
  }

  if(control_docking_probe.getValue()){
    alarmDockingTarget.on();
  }else{
    alarmDockingTarget.off();
  }

  if(cryoO2Q.getValue()>=150 && cryoH2Q.getValue()>=150){
    alarmSPSPress.on();
  }else{
    alarmSPSPress.off();
  }
  alarmACBus1.on();
  alarmAcBus2.on();
  

  if(barVolt.getValue()<5 ){
    alarmMnBusAUndervolt.on();
    alarmMnBusUndervolt.on();
  }else{
    alarmMnBusAUndervolt.off();
    alarmMnBusUndervolt.off();
  }
  
  if(barO2.getValue()<5 ){
    alarmSPSFlngTempHi.on();
  }else{
    alarmSPSFlngTempHi.off();
  }

  if(cws_power.getValue()){
    alarmCW.on();
  }else{
    alarmCW.off();
  }
  
  if(!inco_radio.getValue()){
    alarmFCBusDicnnt.on();
  }else{
    alarmFCBusDicnnt.off();
  }

  if(!pyro_drogue_deploy.getValue()){
    alarmDrogueChute.on();
  }else{
    alarmDrogueChute.off();
  }
  
  if(cryoO2P.getValue() <= 50 || cryoH2P.getValue() <= 50){
    alarmCryoPress.on();
    alert = true;
  }else{
    alarmCryoPress.off();
  }


  if(cryo_o2_fan.getValue() == 2 && timer - cryo_o2_fan.getTime() > TIMEOUT){
    alarmO2FlowHi.on();
  }else{
    alarmO2FlowHi.off();
  }

  if(control_suit_comp.getValue()){
    alarmSuitComp.on();
  }else{
    alarmSuitComp.off();
  }

  if(inco_radio.getValue() && random(0,3)>=1){
    alarmUplinkActivity.on();
  }else{
    alarmUplinkActivity.off();
  }

  alarmBMag1Temp.on();
  alarmBMag2Temp.on();
  alarmCO2PPHI.on();
  alarmPitchGmbl1.on();
  alarmPitchGmbl2.on();
  alarmYawGmbl1.on();
  alarmYawGmbl2.on();
  alarmHgAntScanLimit.on();

  if(!control_glycol_pump.getValue()){
    alarmGlycolTempLow.on();
    alert = true;
  }else{
    alarmGlycolTempLow.off();
  }
  
  if(pyro_csmlv_sep.getValue()){
    alarmCMRCS1.on();
    alarmSMRCSA.on();
    alarmSMRCSB.on();
  }else{
    alarmCMRCS1.off();
    alarmSMRCSA.off();
    alarmSMRCSB.off();
  }
  
  if(pyro_csmlv_sep.getValue()){
    alarmCMRCS2.on();
    alarmSMRCSC.on();
    alarmSMRCSD.on();
  }else{
    alarmCMRCS2.off();
    alarmSMRCSC.off();
    alarmSMRCSD.off();
  }

  if(pyro_apex_cover_jet.getValue()){
    alarmGimbalLock.on();
  }else{
    alarmGimbalLock.off();
  }

  if(alert){
    alarmCrewAlert.on();
  }else{
    alarmCrewAlert.off();
  }
}

void randomNumerics (void)
{
  switch (numericPos) {
    case 0:  
    yaw.threeDigitDisp(random(0,360));
    numericPos = 1;
    break;
    
    case 1:
    pitch.threeDigitDisp(random(0,360));
    numericPos = 2;
    computeCryo();
    break;
    
    case 2:
    roll.threeDigitDisp(random(0,360));
    numericPos = 3;
    break;
    
    case 3:
    ihr.threeDigitDisp(random(0,220));
    numericPos = 4;
    computeCryo();
    break;
    
    case 4:
    abr.threeDigitDisp(random(0,100));
    numericPos = 5;
    break;
    
    case 5:
    ahr.threeDigitDisp(random(0,220));
    numericPos = 0;
    computeCryo();
    break;
  }
}

void scanPots() 
{
  int _brightness = map(analogRead(0), 0, potRef, 0, 10);
  barVolt.potToGraph(_brightness);
  barOhm.potToGraph(map(analogRead(1), 0, potRef, 0, 10));
  barCur.potToGraph(map(analogRead(2), 0, potRef, 0, 10));
  barOx.potToGraph(map(analogRead(3), 0, potRef, 0, 10));
  barSlider.potToGraph(inco_radio.getValue() ? map(analogRead(6), 0, potRef, 0, 5) + map(analogRead(7), 0, potRef, 0, 5) : 0);

  if(brightness == -1 || _brightness != brightness){
      brightness = _brightness;
      FastLED.setBrightness(brightness*10);
    }


  int _volume = map(analogRead(8), 0, potRef, 0, 10);
  if(volume == -1 || _volume != volume){
    volume = _volume;
    player.setVolume(5 * volume);
  }
} // end scanPots()
void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS - 1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for ( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16( i + 7, 0, NUM_LEDS - 1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
void setup ()
{
  #ifdef DEBUG
  Serial.begin(9600);
  #elif
  Serial.begin (115200);  // for debugging via serial terminal on computer
  #endif
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.clear();  // clear all pixel data
  FastLED.show();
  
  player.begin();
  player.setVolume(15);
  
  Wire.begin ();  // to communicate with i2c
  expanderWriteBothPort (IOCON, 0b01100000, expA); // mirror interrupts, disable sequential mode 
  expanderWriteBothPort (IOCON, 0b01100000, expB); // mirror interrupts, disable sequential mode 
  expanderWriteBothPort (IOCON, 0b01100000, expC);
  expanderWriteBothPort (IOCON, 0b01100000, expD);
  expanderWriteBothPort (GPPUA, 0xFF, expA);   // pull-up resistor for switch - both ports
  expanderWriteBothPort (GPPUA, 0xFF, expB);   // pull-up resistor for switch - both ports
  expanderWriteBothPort (GPPUA, 0xFF, expC);
  expanderWriteBothPort (GPPUA, 0xFF, expD);
  expanderWriteBothPort (GPPUB, 0xFF, expA);   // pull-up resistor for switch - both ports
  expanderWriteBothPort (GPPUB, 0xFF, expB);   // pull-up resistor for switch - both ports
  expanderWriteBothPort (GPPUB, 0xFF, expC);
  expanderWriteBothPort (GPPUB, 0xFF, expD);
  expanderWriteBothPort (IOPOLA, 0xFF, expA);  // invert polarity of signal - both ports
  expanderWriteBothPort (IOPOLA, 0xFF, expB);  // invert polarity of signal - both ports
  expanderWriteBothPort (IOPOLA, 0xFF, expC);
  expanderWriteBothPort (IOPOLA, 0xFF, expD);

  matrixA.begin(matAaddy);  // pass in the address of the HT16K33
  matrixB.begin(matBaddy);  // pass in the address of the HT16K33
  matrixC.begin(matCaddy);  // pass in the address of the HT16K33
  matrixD.begin(matDaddy);  // pass in the address of the HT16K33
  matrixE.begin(matEaddy);  // pass in the address of the HT16K33
  matrixF.begin(matFaddy);  // pass in the address of the HT16K33
  matrixG.begin(matGaddy);  // pass in the address of the HT16K33

  matrixA.setBrightness(10);  
  matrixB.setBrightness(10);  
  matrixC.setBrightness(10);  
  matrixD.setBrightness(10);  
  matrixE.setBrightness(10);  
  matrixF.setBrightness(10);  
  matrixG.setBrightness(10);  


  allMatrixOff();

  timer.initClock();
  timer.updateClock();
  clockTimer = millis();
  //lampTestOn();
  //ledOn(matrixE, EBuffer, 4, 4);

  yaw.init();
  roll.init();
  pitch.init();
  ihr.init();
  abr.init();
  ahr.init();
  cryoO2P.init();
  cryoH2P.init();

  //INCO
  inco_radio.setLed(7, 13, BBuffer, &matrixB);
  inco_radio.setPin(52);
  inco_radio.setSounds(42, 42);
  inco_radio.readPin();

 //MISSION
  mission_01.setLed(7, 9, BBuffer, &matrixB);//Mission 1
  mission_01.setSounds(40, 40);
  mission_02.setLed(7, 8, BBuffer, &matrixB);//Mission 2
  mission_02.setSounds(11, 14);
  mission_03.setLed(7, 7, BBuffer, &matrixB);//Mission 3
  mission_03.setSounds(19, 22);
  mission_04.setLed(7, 6, BBuffer, &matrixB);//Mission 4
  mission_04.setSounds(26, 26);
  mission_05.setLed(7, 5, BBuffer, &matrixB);//Mission 5
  mission_05.setSounds(23, 25);
  mission_06.setLed(7, 4, BBuffer, &matrixB);//Mission 6
  mission_06.setSounds(27, 29);
  mission_07.setLed(7, 3, BBuffer, &matrixB);//Mission 7
  mission_07.setSounds(32, 32);
  mission_08.setLed(7, 2, BBuffer, &matrixB);//Mission 8
  mission_08.setSounds(14, 14);
  mission_09.setLed(7, 1, BBuffer, &matrixB);//Mission 9
  mission_09.setSounds(30, 31);
  mission_10.setLed(7, 0, BBuffer, &matrixB);//Mission 10
  mission_10.setSounds(22, 22);


  //CONTROL
  control_docking_probe.setLed(6, 0, CBuffer, &matrixC);//Docking probe
  control_docking_probe.setPin(39);
  control_docking_probe.setSounds(52,52);
  control_glycol_pump.setLed(6, 1, CBuffer, &matrixC);//Glycol pump
  control_glycol_pump.setPin(41);
  control_glycol_pump.setSounds(47,47);
  control_sce_power.setLed(6, 2, CBuffer, &matrixC);//SCE power
  control_sce_power.setPin(43);
  control_sce_power.setSounds(45,45);
  control_waste_dump.setLed(6, 3, CBuffer, &matrixC);//Waste dump
  control_waste_dump.setPin(45);
  control_waste_dump.setSounds(41, 41);
  control_cabin_fan.setLed(6, 4, CBuffer, &matrixC);//Cabin fan
  control_cabin_fan.setPin(47);
  control_cabin_fan.setSounds(55,55);
  control_h2o_flow.setLed(6, 5, CBuffer, &matrixC);//H2O flow
  control_h2o_flow.setPin(49);
  control_h2o_flow.setSounds(54,54);
  control_int_lights.setLed(6, 6, CBuffer, &matrixC);//Int lights
  control_int_lights.setPin(51);
  control_int_lights.setSounds(53,53);
  control_suit_comp.setLed(6, 7, CBuffer, &matrixC);//Suit comp
  control_suit_comp.setPin(53);
  control_suit_comp.setSounds(48,48);

  //CRYOGENIS
  
  cryo_o2_fan.setLed(0, 10, 9, 8, ABuffer, &matrixA);
  cryo_o2_fan.setPins(34, 36);
  cryo_o2_fan.setSounds(-1, 54,54);
  cryo_pumps.setLed(1, 10, 9, 8, ABuffer, &matrixA);
  cryo_pumps.setPins(30, 32);
  cryo_pumps.setSounds(-1, 51,51);
  cryo_h2_fan.setLed(2, 10, 9, 8, ABuffer, &matrixA);
  cryo_h2_fan.setPins(26, 28);
  cryo_h2_fan.setSounds(-1, 54,54);
  cryo_heat.setLed(3, 10, 9, 8, ABuffer, &matrixA);
  cryo_heat.setPins(22, 24);
  cryo_heat.setSounds(-1, 48,48);


  //C&WS
  cws_lamp.setSounds(39, 39);//LAMP
  cws_push.setLed(7, 12, BBuffer, &matrixB);
  cws_push.setSounds(9, 9);
  cws_ack.setSounds(36, 36);//ACK
  cws_power.setSounds(44, 44);//POWER
  cws_mode.setSounds(43,43);//MODE
  
  //ABORT
  abort_push.setLed(7, 11, BBuffer, &matrixB);
  abort_push.setSounds(15, 17);
  //abort_push.setSounds(18, 18);
  abort_arm.setPin(37);
  abort_arm.setSounds(60, 60);
  //pinMode(7, OUTPUT);
  abort_mode_pad.setPin(9);
  abort_mode_1.setPin(12);
  abort_mode_2.setPin(11);
  abort_mode_3.setPin(13);
  abort_mode_4.setPin(8);
  abort_mode_5.setPin(10);
  
  //CAPCOM
  capcom.setLed(7, 14, BBuffer, &matrixB);
  capcom.setSounds(33, 0);
  capcom.setPin(6);
  capcom_key.setPin(50);
  //BOOSTER
  booster_sps.setSounds(59, 59);//SPS
  booster_tei.setSounds(59, 59);//TEI
  booster_tli.setSounds(59, 59);//TLI
  booster_sic.setSounds(59, 59);//SIC
  booster_sii.setSounds(59, 59);//SII
  booster_sivb.setSounds(59, 59);//SIVB
  booster_mi.setSounds(59, 59);//MI
  booster_mii.setSounds(59, 59);//MII
  booster_miii.setSounds(59, 59);//MIII
  
  
  //PYROTECHNICS
  pyro_main_deploy.setSounds(57,57);//MAIN DEPLOY
  pyro_drogue_deploy.setSounds(57,57);//DROGUE DEPLOY
  pyro_csmlv_sep.setSounds(58,58);//CSMLV SEP
  pyro_canard_deploy.setSounds(57,57);//CANARD DEPLOY
  pyro_smcm_sep.setSounds(58,58);//SMCM SEP
  pyro_apex_cover_jet.setSounds(58,58);//APEX COVER JET
  pyro_less_motor_fire.setSounds(56,56);//LES MOTOR FIRE

  //CRYO
  cryoH2P.threeDigitDisp(300);
  cryoO2P.threeDigitDisp(300);
  barH2.potToGraph(10);
  barO2.potToGraph(10);

  //lampTestOn();
//alarm.on();
}  // end of setup


void loop() {
  if(capcom_key.readPin()){
    allMatrixOff();
    reset = true;
    return;
  }
  if(reset){
    reset = false;
    barVolt.potToGraph(0);
    barOhm.potToGraph(0);
    barCur.potToGraph(0);
    barOx.potToGraph(0);
    barH2.potToGraph(0);
    barO2.potToGraph(0);
    barSlider.potToGraph(0);
    timer.initClock();
  }

  if (NUMERICREFRESH < (millis() - numericTimer)) {
    numericTimer = millis();
    randomNumerics();
  }
//  scanButtons();
  if (1000 < (millis() - clockTimer)) {
    clockTimer = millis();
    timer.updateClock();
    matrix.change();
  }
  scanPots();
  
  scanButtons();
  alarms();
  //processSerial();
  sendLight(gCurrentPatternNumber);
  FastLED.show();
  // insert a delay to keep the framerate modest
  FastLED.delay(1000 / FRAMES_PER_SECOND);

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) {
    gHue++;  // slowly cycle the "base color" through the rainbow
  }
}
