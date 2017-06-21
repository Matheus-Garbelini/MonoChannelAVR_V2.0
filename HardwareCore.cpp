//
//
//

#include "HardwareCore.h"

volatile uint32_t milliseconds = 0;
volatile uint8_t sensor1_timeout;
volatile uint8_t sensor2_timeout;
volatile uint8_t ultrassonic_timeout;
volatile uint8_t MotorDirection = 0;
/*OBS: Parametro ISR_NOBLOCK para permitir interrupções aninhadas
(equivalente a sei() no início de rotina) IE é habilitado do registrador STATUS da CPU
*/
//ISR do ultrassonico.
volatile uint8_t ultrassonic_ready = 0;
volatile uint32_t ultrassonic_width;
ISR(INT0_vect) //interrupção SEM suporte a interrupções aninhadas
{
	static uint32_t initial_stamp;
	uint32_t stamp;

	stamp = milliseconds * 1000 + TCNT0 * 4; //milisegundos somados ao valor imediato do contador (4us cada tick)

	if (ULTRASSONIC_PIN_PORT & (1 << ULTRASSONIC_ECHO_PIN)) {
		initial_stamp = stamp;
	}
	else {
		ultrassonic_width = stamp - initial_stamp;
		ultrassonic_ready = 1;
		ultrassonic_timeout = 0;
	}
}

ISR(INT1_vect) //interrupção SEM suporte a interrupções aninhadas
{
	DDRB |= 1 << PB2;
	bitClear(PORTB, PB2);
	OCR0B = TCNT0 + 1;
	TIMSK0 |= 1 << OCIE0B; //habilita interrupção do COMPARADOR B do TIMER0 para limpar pino do trigger;
}

//ISR do Sensor 1 (PD6)
volatile uint8_t sensor1_ready = 0;
volatile uint32_t sensor1_width;
ISR(PCINT1_vect, ISR_NOBLOCK) //interrupção com suporte a interrupções aninhadas
{
	static uint32_t initial_stamp;
	uint32_t stamp;

	stamp = milliseconds * 1000 + TCNT0 * 4; //milisegundos somados ao valor imediato do contador (4us cada tick)

	if (SIGNAL1_PIN_PORT & (1 << SIGNAL1_PIN)) {
		initial_stamp = stamp;
	}
	else {
		sensor1_width = stamp - initial_stamp;
		sensor1_ready = 1;
		sensor1_timeout = 0;
	}
}
//ISR do Sensor 2 (PB4)
volatile uint8_t sensor2_ready = 0;
volatile uint32_t sensor2_width;
ISR(PCINT0_vect, ISR_NOBLOCK) //interrupção com suporte a interrupções aninhadas
{
	static uint32_t initial_stamp;
	uint32_t stamp;

	stamp = milliseconds * 1000 + TCNT0 * 4; //milisegundos somados ao valor imediato do contador (4us cada tick)

	if (SIGNAL2_PIN_PORT & (1 << SIGNAL2_PIN)) {
		initial_stamp = stamp;
	}
	else {
		sensor2_width = stamp - initial_stamp;
		sensor2_ready = 1;
		sensor2_timeout = 0;
	}
}

//ISR da base de tempo em 1ms (precisão de 4us)
ISR(TIMER0_COMPA_vect) //ISR com alta prioridade (interrupções aninhadas não permitidas)
{
	//pré incremento é mais rápido
	++milliseconds;
	++sensor1_timeout;
}

ISR(TIMER0_COMPB_vect, ISR_NOBLOCK) //ISR para limpar pino de trigger do ultrassonico (soporte a interrupção aninhada)
{
	EN2Write(1);
	EN1Write(1);
	bitClear(TIMSK0, OCIE0B);
}

ISR(TIMER1_COMPA_vect) //ISR para limpar pino de trigger do ultrassonico (soporte a interrupção aninhada)
{
	if (MotorDirection == 0) {
		bitClear(IN1_PORT, IN1_PIN);
		bitSet(IN1_PORT, IN1_PIN);
	}
	else
	{
		bitClear(IN2_PORT, IN2_PIN);
		bitSet(IN2_PORT, IN2_PIN);
	}
}

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) //ISR para limpar pino de trigger do ultrassonico (soporte a interrupção aninhada)
{
	if (MotorDirection == 0)
		bitClear(EN1_PORT, EN1_PIN);
	else
		bitClear(EN2_PORT, EN2_PIN);
}

ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //ISR para limpar pino de trigger do ultrassonico (soporte a interrupção aninhada)
{
	if (MotorDirection == 0)
		bitSet(EN1_PORT, EN1_PIN);
	else
		bitSet(EN2_PORT, EN2_PIN);
}

void delay_ms(uint16_t wait_stamp) {
	uint32_t initial_stamp;
	initial_stamp = milliseconds;
	while (milliseconds - initial_stamp <= wait_stamp); //não faça nada
}

void delay_us(uint16_t wait_stamp) {
	uint32_t initial_stamp;
	initial_stamp = microseconds;
	while (microseconds - initial_stamp <= wait_stamp); //não faça nada
}

uint16_t sensor1_getValue()
{
	static uint16_t value;
	if (sensor1_ready) {
		sensor1_ready = 0;
		value = sensor1_width;
		if (value > SENSOR1_TIMEOUT * 1000) value = 0;
	}
	else if (sensor1_timeout > SENSOR1_TIMEOUT) {
		value = 0;
		sensor1_timeout = 0;
	}

	return value;
}

uint16_t sensor2_getValue() {
	static uint16_t value;
	if (sensor2_ready) {
		sensor2_ready = 0;
		value = sensor2_width;
		if (value > SENSOR2_TIMEOUT * 1000) value = 0;
	}
	else if (sensor2_timeout > SENSOR2_TIMEOUT) {
		value = 0;
		sensor2_timeout = 0;
	}

	return value;
}

uint16_t ultrassonic_getValue() {
	static uint16_t value;

	UltrassonicWrite(1); //seta pino de trigger do ultrassonico
						 //configura comparador B do timer 0 para disparar interrupção daqui a 12 us
						 //*necessário de acordo com especificações
	OCR0B = (TCNT0 + 3) % 249;

	//UltrassonicWrite(0); este comando ocorre na interrupção "TIMER0_COMPB_vect"

	if (ultrassonic_ready) {
		ultrassonic_ready = 0;
		value = ultrassonic_width;
		if (value > ULTRASSONIC_TIMEOUT * 1000) value = 0;
	}
	else if (ultrassonic_timeout > ULTRASSONIC_TIMEOUT) {
		value = 0;
		ultrassonic_timeout = 0;
	}

	return value;
}