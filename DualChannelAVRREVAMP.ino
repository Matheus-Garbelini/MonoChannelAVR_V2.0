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
	delay_ms(1000);
	LedWrite(0);
}

void loop() {
	robotProcess();
}