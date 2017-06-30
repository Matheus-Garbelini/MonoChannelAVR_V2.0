// MotorControlPUC.h

#ifndef _MOTORCONTROLPUC_h
#define _MOTORCONTROLPUC_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <inttypes.h>
#include "avr/wdt.h"
#include <EEPROM.h>

#include "Config.h"
#include "HardwareConfig.h"
#include "HardwareCore.h"

#pragma region Functions
void setupBoard();
void drive();             // Procedure responsible for convert receiver commands into H-bridge drive (movement)
void robotProcess();
void fail_safe();           // Procedure to check if the communication was lost
void calibration();         // Procedure responsible for calibrate radio

void motor1PWM(uint8_t pwm); //muda pwm de motor1 (com deadtime) pinos 3 e 11
void motor2PWM(uint8_t pwm); //muda pwm de motor2 (com deadtime) pinos 9 e 10
void motor2PWMRight(uint8_t pwm);
void motor2PWMLeft(uint8_t pwm);
void motor2Stop();
uint16_t map_channel(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max); //mapeia valores 16bits

//----------------------------------------------
//funções para capturar valores dos sensores (largura do pulso)

//debugação
void print_sensors();
#pragma endregion

#endif
