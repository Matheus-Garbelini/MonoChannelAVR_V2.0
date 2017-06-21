/*DeadTimeV2
   Atualização do controle de motores
   - SUPORTE A 2 CANAIS (PORÉM UTILIZADO SOMENTE 1 NA PLACA)
   - SUPORTE A SENSOR ULTRASSONICO
   - PROGRAMAÇÃO TOTALMENTE NÃO BLOQUEADA NA LEITURA DE SINAIS E ULTRASSONICO (Canais e ultrassonico orientados a interrupções)
   - TIMEOUT SEPARADO PARA CADA CANAL/SENSOR
   - DEADTIME DE APROXIMADAMENTE 600ns acrescentado em PWMs complementares
*/

#include <Arduino.h>
#include "MotorControlPUC.h"

void setup() {
	setupBoard();
	LedWrite(1);
	DDRD |= 1 << 5;
	bitSet(DDRB, EN2_PIN);
}

uint8_t i = 0;
void loop() {
	static uint32_t last_milliseconds = milliseconds;
	//robotProcess();
	motor2PWM(i);
	bitClear(PORTB, 3);
	bitSet(PORTB, 3);
	if (milliseconds - last_milliseconds > 3) {
		last_milliseconds = milliseconds;

		i++;
	}
}