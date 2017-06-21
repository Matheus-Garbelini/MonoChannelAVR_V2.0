// HardwareCore.h

#ifndef _HARDWARECORE_h
#define _HARDWARECORE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Config.h"

#pragma region Macros
//----------------Macros-----------------------
//macros de operação bitwise
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
//    Timing
#define microseconds (milliseconds*1000 + TCNT0*4)      //macro para requisitar tempo em microsegundos (precisão de 4us)
//    GPIO control
#define IN1Write(bitvalue) (bitvalue ? bitSet(IN1_PORT,IN1_PIN) : bitClear(IN1_PORT, IN1_PIN))   //pwm da meia ponte 1 do motor 1 (pino 3)
#define IN2Write(bitvalue) (bitvalue ? bitSet(IN2_PORT,IN2_PIN) : bitClear(IN2_PORT, IN2_PIN))   //pwm da meia ponte 2 do motor 1 (pino 11)

#define EN1Write(bitvalue) (bitvalue ? bitSet(EN1_PORT,EN1_PIN) : bitClear(EN1_PORT, EN1_PIN)) //escreve no Enable da Meia ponte 1 (motor 1) pino 7
#define EN2Write(bitvalue) (bitvalue ? bitSet(EN2_PORT,EN2_PIN) : bitClear(EN2_PORT, EN2_PIN)) //escreve no Enable do Meia ponte 2 (motor 2) pino 8

#define LedWrite(bitvalue) (bitvalue ? bitSet(LED_STATUS_PORT,LED_STATUS_PIN) : bitClear(LED_STATUS_PORT, LED_STATUS_PIN)) // escreve no pino do LED indicativo

#define UltrassonicWrite(bitvalue) (bitvalue ? bitSet(ULTRASSONIC_PORT,ULTRASSONIC_TRIGGER_PIN) : bitClear(ULTRASSONIC_PORT, ULTRASSONIC_TRIGGER_PIN)) // escreve no pino trigger do ultrassonico

#define CalibrateRead() analogRead(A7) //le botão de calibração

#define EnableMotorDriverRefresh(enabled) (enabled ? bitSet(TIMSK1,OCIE1A):bitClear(TIMSK1,OCIE1A))

#ifdef SERIAL_MONITOR
#define MONITOR_PRINTLN(x) do {Serial.println(x);} while(0)
#define MONITOR_PRINT(x) do {Serial.print(x);} while (0)
#else
#define MONITOR_PRINTLN(x) do {} while(0)
#define MONITOR_PRINT(x) do {} while(0)
#endif
//----------------------------------------------
#pragma endregion

extern volatile uint32_t milliseconds;

void delay_ms(uint16_t wait_stamp);
void delay_us(uint16_t wait_stamp);

uint16_t sensor1_getValue();
uint16_t sensor2_getValue();
uint16_t ultrassonic_getValue();
extern volatile uint8_t MotorDirection;

#endif
