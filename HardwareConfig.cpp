//
//
//

#include "HardwareConfig.h"

uint16_t ch1_min = 0;
uint16_t ch1_max = 0;
uint16_t ch2_min = 0;
uint16_t ch2_max = 0;

#pragma region Interrupts_Channels

void configInterrupt_Ultrassonic() {
	//------------------------------Configuração de interrupções----------------------------//
	//---------------------Interrupção do Ultrassonico (PD2)--------------------------------//
	//Interrupção IN0 configurada para qualquer mudança de estado
	EICRA |= 0 << ISC01 | 1 << ISC00; //EICRA |= 1 << 1 | 1 << 0;
	//Habilita interrupção externo IN0
	EIMSK |= 1 << INT0; //EIMSK |= 1 << 0;
	TIMSK0 |= 1 << OCIE0B; //habilita interrupção do COMPARADOR B do TIMER0 para limpar pino do trigger;
	//--------------------------------------------------------------------------------------//
}

void configInterrupt_Channel1() {
	//---------------------Interrupção do Sinal 1 (PD6 - 6)--------------------------------//
	//habilita interrupção de mudança de estado para PORTD
	PCICR |= 1 << PCIE1; // PCICR |= 1 << 2;
	//habilita pino PD2 para interrupção de mudança de estado
	PCMSK1 |= 1 << SIGNAL1_PCINT; //PCMSK2 |= 1 << 6;
	//--------------------------------------------------------------------------------------//
}

void configInterrupt_Channel2() {
	//---------------------Interrupção do Sinal 2 (PB4 - 12)--------------------------------//
	//habilita interrupção de mudança de estado para PORTD
	PCICR |= 1 << PCIE0; // PCICR |= 1 << 0;
	//habilita pino PD2 para interrupção de mudança de estado
	PCMSK0 |= 1 << PCINT4; //PCMSK2 |= 1 << 4;
	//--------------------------------------------------------------------------------------//
}

#pragma endregion

#pragma region Timers

/*Frequência base do PWM em modo phase correct (dual slope) é 31372,55 Hz
Pois F_pwm = FOSC/(PreScaler*2*255)
Tabela de frequencia para Fosc = 16Mhz
TCCR2A |= (0 << CS22 | 0 << CS21 | 1 << CS21); - 31372,55 Hz - Div 1
TCCR2A |= (0 << CS22 | 1 << CS21 | 0 << CS21); - 3921,57 Hz - Div 8
TCCR2A |= (0 << CS22 | 1 << CS21 | 1 << CS21); - 980,39 Hz - Div 32
TCCR2A |= (1 << CS22 | 0 << CS21 | 0 << CS21); - 490,2 Hz - Div 64
TCCR2A |= (1 << CS22 | 0 << CS21 | 1 << CS21); - 245,098 Hz - Div 128
TCCR2A |= (1 << CS22 | 1 << CS21 | 0 << CS21); - 122,55 Hz - Div 256
TCCR2A |= (1 << CS22 | 1 << CS21 | 1 << CS21); - 30,64 Hz - Div 1024
*/

void configTimer0() {
	//-------------------Configuração Timer 0 como base de tempo de 1ms (4us de precisão) -------------------//
	TCCR0A = 0; //Limpa registrador de controle A do timer 0
	TCCR0B = 0; //Limpa registrador de controle B do timer 0
	TIMSK0 = 0; //Limpa registrador de interrupções do timer 0
	//--------------------Configuração Prescaler------------------------//
	//Prescaler em 64
	TCCR0B |= (0 << CS12 | 1 << CS11 | 1 << CS10); //TCCR2B = 0 << 2 | 0 << 1 | 1 << 0;
	//-------------------Modo de contagem (waveform)--------------------//
	//Configura Wave Form Generator para operar em CTC (Clear Timer on Compare match)
	TCCR0A |= (0 << WGM02 | 1 << WGM01 | 0 << WGM00); //TCCR0A |= (0 << 2 | 0 << 1 | 1 << 0);
	OCR0A = 249; //4us*(250-1) = 1ms (*verificado experimentalmente)
	TIMSK0 |= 1 << OCIE0A; //TIMSK0 |= 1 << 1 | 1 << 2; //habilita interrupção para base de tempo
};
void configTimer1() {
	//-------------------Configuração Timer 1 como PWM (Pinos 9 e 10) -------------------//
	TCCR1A = 0; //Limpa registrador de controle A do timer 1
	TCCR1B = 0; //Limpa registrador de controle B do timer 1
	//--------------------Configuração Prescaler------------------------//
	//Prescaler em 1 (sem divisão)

	 //-------------------Modo de contagem (waveform)--------------------//
	//Configura Wave Form Generator para o perar em modo phase correct pwm (dual slope counter)
	TCCR1B |= (0 << WGM13) | (1 << WGM12);
	TCCR1A |= (0 << WGM11) | (1 << WGM10);
	//TCCR1A |= (1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0);
	TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); //TCCR2B = 0 << 2 | 0 << 1 | 1 << 0;
	//TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);
	//---------------------------------------------------------------------------------------//
};
void configTimer2() {
	//-------------------Configuração Timer 2 como PWM (Pinos 3 e 11) -------------------//
	TCCR2A = 0; //Limpa registrador de controle A do timer 2
	TCCR2B = 0; //Limpa registrador de controle B do timer 2
	//--------------------Configuração Prescaler------------------------//
	//Prescaler em 1 (sem divisão)
	TCCR2B |= (1 << CS22 | 0 << CS21 | 0 << CS20); //TCCR2B = 0 << 2 | 0 << 1 | 1 << 0;
	//-------------------Modo de contagem (waveform)--------------------//
	//Fast PWM TOP=OCRA
	TCCR2A |= (1 << WGM21 | 1 << WGM20); //TCCR2A |= (0 << 2 | 0 << 1 | 1 << 0);
	//-----------------Configura polaridade PWM-------------------------//
	/*modo invertido em canal A (pino11) */
	//TCCR2A |= (1 << COM2A1 | 0 << COM2A0); //TCCR2A |= (1 << 7 | 1 << 6);
	//TCCR2A |= (1 << COM2B1 | 1 << COM2B0); //TCCR2A |= (1 << 7 | 1 << 6);
	OCR2A = 254;
	TCNT2 = TCNT1 + 1;
	/*modo nao invertido em canal B (pino3) */
	//TCCR2A |= (0 << COM2B1 | 0 << COM2B0); //TCCR2A |= (1 << 5 | 0 << 4);
	//---------------------------------------------------------------------------------------//
	//TIMSK2 |= (1 << TOIE2) | (1 << OCIE2A);
};
#pragma endregion

#pragma region GPIO
void configGPIO_Ultrassonic() {
	bitClear(ULTRASSONIC_DDR, ULTRASSONIC_ECHO_PIN); //configura pino 2 (RD2) como entrada;
	bitSet(ULTRASSONIC_PORT, ULTRASSONIC_ECHO_PIN); //configura pino 2 (RD2) como entrada pullup
	bitSet(ULTRASSONIC_DDR, ULTRASSONIC_TRIGGER_PIN); //configura pino 4 (RD4) como saída
	bitClear(ULTRASSONIC_PORT, ULTRASSONIC_TRIGGER_PIN); //limpa pino 4 (RD4)
}

void configGPIO_Channel1() {
	bitClear(SIGNAL1_DDR, SIGNAL1_PIN); //configura pino 6 (RD6) como entrada
	bitSet(SIGNAL1_PORT, SIGNAL1_PIN); //configura pino 6 (RD6) como entrada pullup
}

void configGPIO_Channel2() {
	bitClear(SIGNAL1_DDR, SIGNAL2_PIN); //configura pino 12 (RB4) como entrada
	bitSet(SIGNAL2_PORT, SIGNAL2_PIN); //configura pino 12 (RD4) como entrada pullup
}

void configGPIO_Calibrate() {
	//bitClear(CALIBRATE_DDR, CALIBRATE_PIN); //pino A2 (PC2) como entrada
	//bitSet(CALIBRATE_PORT, CALIBRATE_PIN); //pino A2 (PC2) com pullup
}

void configGPIO_MOTOR1_PWM() {
	bitSet(IN3_DDR, IN3_PIN); //pino 9 (RB1)
	bitClear(IN3_PORT, IN3_PIN); //limpa
	bitSet(IN4_DDR, IN4_PIN); //pino 10 (RB2)
	bitClear(IN4_PORT, IN4_PIN); //limpa
}

void configGPIO_MOTOR1_ENABLE() {
	bitSet(EN2_DDR, EN2_PIN);
#ifdef INVERTED_EN
	bitClear(EN2_PORT, EN2_PIN);
#else
	bitSet(EN2_PORT, EN2_PIN); //seta pino (desliga driver 2 do motor 2)
#endif // INVERTED_EN
}

void configGPIO_MOTOR2_PWM() {
	bitSet(IN1_DDR, IN1_PIN); //pino 3 (RD3)
	bitClear(IN1_PORT, IN1_PIN); //limpa
	bitSet(IN2_DDR, IN2_PIN); //pino 11 (RB3)
	bitClear(IN2_PORT, IN2_PIN);//limpa
}

void configGPIO_MOTOR2_ENABLE() {
	bitSet(EN1_DDR, EN1_PIN);
#ifdef INVERTED_EN
	bitClear(EN1_PORT, EN1_PIN);
#else
	bitSet(EN1_PORT, EN1_PIN); //seta pino (desliga driver 1 do motor 1)
#endif // INVERTED_EN

	bitSet(EN2_DDR, EN2_PIN);
#ifdef INVERTED_EN
	bitClear(EN2_PORT, EN2_PIN);
#else
	bitSet(EN2_PORT, EN2_PIN); //seta pino (desliga driver 1 do motor 1)
#endif // INVERTED_EN
}

#pragma endregion

#pragma region HARDWARE_SETUP
void setupTimeBase() {
	configTimer0(); //configura base de tempo de 1ms (4us de precisão)
}

void setupUltrassonic() {
	configGPIO_Ultrassonic();
	configInterrupt_Ultrassonic();
}

void setupChannel1() {
	configGPIO_Channel1();
	configInterrupt_Channel1();
}

void setupChannel2() {
	configGPIO_Channel2();
	configInterrupt_Channel2();
}

void setupCalibrate() {
	configGPIO_Calibrate();
}

void setupEEPROM()
{
	//carrega valores anteriormentes salvas na EEPROM
	ch1_min = constrain(map(EEPROM.read(addr_low1), 0, 255, 0, 2000), 0, 2000); // Retrieve EEPROM value for minimal channel value
	ch1_max = constrain(map(EEPROM.read(addr_high1), 0, 255, 0, 2000), 0, 2000); // Retrieve EEPROM value for maximum channel value
	ch2_min = constrain(map(EEPROM.read(addr_low2), 0, 255, 0, 2000), 0, 2000); // Retrieve EEPROM value for minimal channel value
	ch2_max = constrain(map(EEPROM.read(addr_high2), 0, 255, 0, 2000), 0, 2000); // Retrieve EEPROM value for maximum channel value
}

void setupMotor1() {
	configGPIO_MOTOR1_ENABLE(); //configura pino Enable
	configGPIO_MOTOR1_PWM(); //configura pinos do pwm
	configTimer1(); //habilita pwm
}

void setupMotor2() {
	configGPIO_MOTOR2_ENABLE(); //configura pino Enable
	configGPIO_MOTOR2_PWM(); //configura pinos do pwm
	configTimer2(); //habilita pwm
	configTimer1();
}

void beckup() {
	//-------------------Configuração Timer 1 como PWM (Pinos 9 e 10) -------------------//
	TCCR1A = 0; //Limpa registrador de controle A do timer 1
	TCCR1B = 0; //Limpa registrador de controle B do timer 1
				//--------------------Configuração Prescaler------------------------//
				//Prescaler em 1 (sem divisão)

				//-------------------Modo de contagem (waveform)--------------------//
				//Configura Wave Form Generator para o perar em modo phase correct pwm (dual slope counter)
	TCCR1B |= (0 << WGM13) | (1 << WGM12);
	TCCR1A |= (0 << WGM11) | (1 << WGM10);
	TCCR1A |= (1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0);
	OCR1AL = 128;
	OCR1BL = 128;
	IN1_DDR |= (1 << IN1_PIN);
	IN2_DDR |= (1 << IN2_PIN);
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); //TCCR2B = 0 << 2 | 0 << 1 | 1 << 0;
													   //---------------------------------------------------------------------------------------//
}

#pragma endregion