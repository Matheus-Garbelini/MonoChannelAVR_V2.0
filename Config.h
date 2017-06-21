#pragma once

#define F_CPU 16000000

#pragma region Timings
//--------------Dead Time-------------------------
#define DEADTIME_OFFSET  9
//--------------Time outs-----------------------
#define SENSOR1_TIMEOUT 60UL      //timout de canal 1 em milisegundos
#define SENSOR2_TIMEOUT 60UL      //timout de canal 2 em milisegundos
#define ULTRASSONIC_TIMEOUT 60UL  //timout de ultrassonico em milisegundos
#pragma endregion

#pragma region Pins
//Pins definition
//Motor 2
#define IN1_PORT    PIND    // PWM pin 3
#define IN1_DDR     DDRD    // PWM pin 3
#define IN1_PIN     PD3   // PWM pin 3

#define IN2_PORT    PINB    // PWM pin 11
#define IN2_DDR     DDRB    // PWM pin 11
#define IN2_PIN     PB3   // PWM pin 11
//Motor 1
#define IN3_PORT    PINB    // PWM pin 9
#define IN3_DDR     DDRB    // PWM pin 9
#define IN3_PIN     PB1   // PWM pin 9

#define IN4_PORT    PINB    // PWM pin 10
#define IN4_DDR     DDRB    // PWM pin 10
#define IN4_PIN     PB2   // PWM pin 10

#define EN1_PORT  PORTD
#define EN1_DDR   DDRD
#define EN1_PIN   PD5 // Enable bridge`s driver 1 (just for version with 2 bridges)
#define EN2_PORT  PORTB
#define EN2_DDR   DDRB
#define EN2_PIN   PB4 // Enable bridge`s driver 1 (just for version with 2 bridges)

//Calibration
#define CALIBRATE_PORT    PORTC   // Calibration pin
#define CALIBRATE_PIN_PORT  PINC  // Calibration pin
#define CALIBRATE_DDR     DDRC  // Calibration pin
#define CALIBRATE_PIN     PC2   // Calibration pin

#define LED_STATUS_PORT PORTB // Status LED pin
#define LED_STATUS_DDR  DDRB  // Status LED pin
#define LED_STATUS_PIN  PB0 // Status LED pin

//Sensors
//Canal 1 - A0
#define SIGNAL1_PORT    PORTC
#define SIGNAL1_PIN_PORT  PINC
#define SIGNAL1_DDR     DDRC
#define SIGNAL1_PIN     PC0
#define SIGNAL1_PCINT     PCINT8
//Canal 2 - Pino D12
#define SIGNAL2_PORT    PORTB
#define SIGNAL2_PIN_PORT  PINB
#define SIGNAL2_DDR     DDRB
#define SIGNAL2_PIN     PB4
#define SIGNAL2_PCINT     PCINT4

#define ULTRASSONIC_PORT    PORTD
#define ULTRASSONIC_PIN_PORT    PIND
#define ULTRASSONIC_DDR     DDRD
#define ULTRASSONIC_ECHO_PIN  PD2
#define ULTRASSONIC_TRIGGER_PIN PD4
//----------------------------------------------------------------------------------------------------

#pragma endregion

#pragma region EEPROM_Settings
// EEPROM Addressing
#define addr_low1 250       // EEPROM Address for minimal channel value (0<=value<=512)
#define addr_high1 255      // EEPROM Address for maximum channel value (0<=value<=512)
#define addr_low2 260       // EEPROM Address for minimal channel value (0<=value<=512)
#define addr_high2 265      // EEPROM Address for maximum channel value (0<=value<=512)
#pragma endregion

#pragma region Settings
// General settings
#define dead_band 50      // Define dead band, interval (+-) of stick movement before start to drive motors
#define ENABLE_FAILSAFE
#define fail_safe_limit 500       // Define fail_safe limit (smaller values are considered as disconnected, usually extreme values are 900<ch<2200)
#define pwm_max 238
#define pwm_min 20
#define pwm_half 127
#define deadtime_min  pwm_min + DEADTIME_OFFSET
#define INVERTED_EN
//#define PWM_INVERTED
//#define SERIAL_MONITOR
//#define WATCHDOG_TIMER
#define ENABLE_EEPROM_CHECK
//#define MOTOR1_ENABLED
#define MOTOR2_ENABLED
#pragma endregion

#pragma region Functionalities
// Functionalities
#define serial_monitor  0   // Serial monitor ("debbug")              0 for off, >1 for ON
#define watchdog_timer 0    // Force hardware reset when µC freezes   0 for off, >1 for ON
#define serial_control 0      // Control the board by serial AT commands  0 for off, >1 for ON
#define eeprom_check 1    // Shows the stored eeprom values for "low" and "high" values (only works if serial_monitor is enabled) 0 for off, >1 for ON
//----------------------------------------------
#pragma endregion