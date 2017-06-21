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
	EnableMotorDriverRefresh(1);
}

void loop() {
	for (uint8_t i = 0; i < 255; i++)
	{
		motor2PWM(i);
		delay_ms(5);
	}
	for (uint8_t i = 255; i > 0; i--)
	{
		motor2PWM(i);
		delay_ms(5);
	}
}