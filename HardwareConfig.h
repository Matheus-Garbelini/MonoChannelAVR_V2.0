// HardwareConfig.h

#ifndef _HARDWARECONFIG_h
#define _HARDWARECONFIG_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Config.h"
#include "MotorControlPUC.h"

//Exportando variáveis para uso em funções externas
extern uint16_t ch1_min;
extern uint16_t ch1_max;
extern uint16_t ch2_min;
extern uint16_t ch2_max;
//------------------

//funções públicas

//--------------

#endif

void setupTimeBase();

void setupUltrassonic();

void setupChannel1();

void setupChannel2();

void setupCalibrate();

void setupEEPROM();

void setupMotor1();

void setupMotor2();

void setupDeadTime();