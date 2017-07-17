#include "MotorControlPUC.h"
#include <Arduino.h>
#include <EEPROM.h>

//-------------Global variables-----------------
static uint16_t ch1, ch2;              // Storage receiver channel 1 read value
extern uint16_t ch1_min;
extern uint16_t ch1_max;
extern uint16_t ch2_min;
extern uint16_t ch2_max;
uint8_t status = 0;              // Indicate when is any change in rotation direction
//------------Private Funcions-----------------
void configBoard();

void setupBoard() {
	configBoard(); //Configura todos os periféricos da placa

#ifdef SERIAL_MONITOR
	Serial.begin(115200);             // Start serial communication
	Serial.println("ON!");            // Power on message
#endif // SERIAL_MO

	ch1 = sensor1_getValue();
	//ch2 = sensor2_getValue(); // Read receiver control channe

							  /********VISUALISAR VALORES ARMAZENADOS NA EEPROM***********/
#ifdef ENABLE_EEPROM_CHECK
#ifdef SERIAL_MONITOR
	//Channel 1
	Serial.print("ch1_min: ");
	Serial.print(ch1_min);
	Serial.print(" - ");
	Serial.print("ch1_max: ");
	Serial.println(ch1_max);

	// Channel 2
	Serial.print("ch2_min: ");
	Serial.print(ch2_min);
	Serial.print(" - ");
	Serial.print("ch2_max: ");
	Serial.println(ch2_max);
#endif
#endif // ENABLE_EEPROM_CHECK

	//delay_ms(1000); //1s de espera para estabilizar circuito

#ifdef WATCHDOG_TIMER
	wdt_enable(WDTO_2S);
#endif
}

//----------------------------------------------
void configBoard() {
	//inicializa todas as funções da placa
	setupTimeBase();
	setupChannel1();
	//setupChannel2();
	//setupUltrassonic();
	//setupCalibrate();
	setupEEPROM();
#ifdef MOTOR1_ENABLED
	setupMotor1();
#endif // DEBUG

#ifdef MOTOR2_ENABLED
	setupMotor2();
#endif // DEBUG

	LED_STATUS_DDR |= (1 << LED_STATUS_PIN);
}

void motor1PWM(uint8_t pwm) {
	//	uint8_t pwm1, pwm2;
	//	if (pwm < deadtime_min) pwm1 = deadtime_min;
	//	else if (pwm > pwm_max) pwm1 = pwm_max;
	//	else pwm1 = pwm;
	//	pwm2 = pwm1 - DEADTIME_OFFSET;
	//
	//#ifndef PWM_INVERTED
	//	OCR1A = pwm2;
	//	OCR1B = pwm1;
	//#else
	//	OCR1A = pwm1;
	//	OCR1B = pwm2;
	//#endif // PWM_INVERTED
}

void motor2PWM(uint8_t pwm) {
	MotorDirection = 1;
	EN1Write(0);
	IN1Write(0);
	OCR1A = pwm;
}

void motor2Stop() {
	TCCR2A &= 0x0F;
	TIMSK1 = 0;
	EN2Write(1);
	IN2Write(0);
	EN1Write(1);
	IN1Write(0);
}

void motor2PWMRight(uint8_t pwm) {
	static double _difference = 0;
	static double Output_Activation = 255.0;
	static uint32_t lastTime = milliseconds;

	if (milliseconds - lastTime > 10) {
		_difference = pwm - Output_Activation;
		Output_Activation += (_difference*0.065);
		lastTime = milliseconds;
	}

	MotorDirection = 1;
	TCCR2A |= (1 << COM2A1 | 0 << COM2A0); //TCCR2A |= (1 << 7 | 1 << 6);
	EN1Write(0);
	IN1Write(0);
	TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);
	OCR1A = Output_Activation;
}

void motor2PWMLeft(uint8_t pwm) {
	static double _difference = 0;
	static double Output_Activation = 255.0;
	static uint32_t lastTime = milliseconds;

	if (milliseconds - lastTime > 10) {
		_difference = pwm - Output_Activation;
		Output_Activation += (_difference*0.065);
		lastTime = milliseconds;
	}

	MotorDirection = 0;
	TCCR2A |= (1 << COM2B1 | 1 << COM2B0); //TCCR2A |= (1 << 7 | 1 << 6);
	EN2Write(0);
	IN2Write(0);
	TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);
	OCR1A = Output_Activation;
}

void calibration() {
	//-------------Calibrate----------------------

	//------------Turn motor off!-----------------

	motor2Stop(); // Turn motor off!
						  //-----Calibrate serial initialize flag-------
	if (serial_monitor) {
		Serial.print("5, ");
		delay_ms(50);
		Serial.print("4, ");
		delay_ms(50);
		Serial.print("3, ");
		delay_ms(50);
		Serial.print("2, ");
		delay_ms(50);
		Serial.println("1.. ");
		delay_ms(50);
	}

#ifdef WATCHDOG_TIMER
	wdt_reset();                              //  I am still alive baby! (Says to Watchdog that everything is ok!)
#endif
	MONITOR_PRINTLN("Calibrate ON");         // Print calibration flag

	LedWrite(0);   // Set LED off

	fail_safe();      // Check if is a receiver connected, eventhough fail_safe isn`t enabled

					  //-----Blink 3 times to indicate Start-------
	for (int i = 0; i < 3; i++) {
		MONITOR_PRINT(" .");
		LedWrite(1);
		delay_ms(100);
		LedWrite(0);
		delay_ms(100);
	}

	MONITOR_PRINTLN();               // Print /CR /LF
									 //--Here you should move radio sticks-------
									 // It will realize 250 measurements and define maximum, minimum and middle stick position

	ch1_max = 800;                  // Set a low value to be replaced while calibrating
	ch1_min = 2000;                 // Set a high value to be replaced while calibrating
	ch2_max = 800;                  // Set a low value to be replaced while calibrating
	ch2_min = 2000;                 // Set a high value to be replaced while calibrating

	for (int i = 0; i < 350; i++) {
#ifdef WATCHDOG_TIMER
		wdt_reset();                            //  I am still alive baby! (Says to Watchdog that everything is ok!)
#endif // WATCHDOG_TIMER

		ch1 = sensor1_getValue();
		//ch2 = sensor2_getValue();     // Read receiver control channel

		ch1_min = (ch1 < ch1_min) ? ch1 : ch1_min;  // Look if new value is smaller than old one
		ch1_max = (ch1 > ch1_max) ? ch1 : ch1_max;  // Look if new value is bigger than old one

		//ch2_min = (ch2 < ch2_min) ? ch2 : ch2_min;  // Look if new value is smaller than old one
		//ch2_max = (ch2 > ch2_max) ? ch2 : ch2_max;  // Look if new value is bigger than old one

													// Turn led on during calibration
		if (i < 250)
			LedWrite(1);        // Set LED on
		else
			LedWrite(0);        // Set LED off

		delay_ms(15);    // Delay para leitura durar 3 segundos no total
	}

	(ch1_max == 800) ? 2000 : ch1_max;      // Check if a calibration occurred with out any radio signal, then ignore it..
	(ch1_min == 2000) ? 800 : ch1_min;      // Check if a calibration occurred with out any radio signal, then ignore it..

	//(ch2_max == 800) ? 2000 : ch2_max;      // Check if a calibration occurred with out any radio signal, then ignore it..
	//(ch2_min == 2000) ? 800 : ch2_min;      // Check if a calibration occurred with out any radio signal, then ignore it..

	EEPROM.write(addr_low1, constrain(map(ch1_min, 0, 2000, 0, 255), 0, 255)); // Write into EEPROM the new channel smallest value
	EEPROM.write(addr_high1, constrain(map(ch1_max, 0, 2000, 0, 255), 0, 255)); // Write into EEPRO the new channel biggest value
	//EEPROM.write(addr_low2, constrain(map(ch2_min, 0, 2000, 0, 255), 0, 255)); // Write into EEPROM the new channel smallest value
	//EEPROM.write(addr_high2, constrain(map(ch2_max, 0, 2000, 0, 255), 0, 255)); // Write into EEPRO the new channel biggest value

																				//-----Blink 3 times to indicate the END-----
	for (int i = 0; i < 3; i++) {
		LedWrite(1);
		delay_ms(100);
		LedWrite(0);
		delay_ms(100);
	}

	MONITOR_PRINTLN("Calibrate OFF");          // Print calibration end flag

	while (CalibrateRead() == 0) {              //  Wait for the calibration jumper be tooked of
#ifdef WATCHDOG_TIMER
		wdt_reset();                              //  I am still alive baby! (Says to Watchdog that everything is ok!)
#endif // WATCHDOG_TIMER

		if (milliseconds % 1000 == 0) {
			LedWrite(1);
			delay_ms(50);
			LedWrite(0);
		}
	}

	LedWrite(0);  // Set LED off
}

void fail_safe() {
	// Local Variables
	int fail = 0;                         // fail_safe flag

	if (ch1 < fail_safe_limit)                  // Check if channel 1 is disconnected (usually extreme values are 900<ch1<2200)
		fail = 1;                         // Set fail_safe flag ON

	if (fail) {                         // If on fail_safe state
		while (fail) {                      // While on fail_safe state
#ifdef WATCHDOG_TIMER
			wdt_reset();                                //  I am still alive baby! (Says to Watchdog that everything is ok!)
#endif // DEBUG

			ch1 = sensor1_getValue();          // Read receiver control channel
			//ch2 = sensor2_getValue();           // Read receiver control channel

			if (ch1 > fail_safe_limit)              // Check if channel 1 is connected (usually extreme values are 900<ch1<2200)
				fail = 0;                     // Set fail_safe flag OFF

#ifdef SERIAL_MONITOR
											  // If serial control is enable (AT commands)
											  //at_control();
#endif // MONI

#ifdef ENABLE_FAILSAFE
			fail = 0;
#endif // !1

			LedWrite(1);              // Set LED on

#ifdef WATCHDOG_TIMER
			wdt_reset();
#endif

			motor2Stop();

#ifdef SERIAL_MONITOR
			Serial.print("Channel: ");              // Print channel value
			Serial.print(ch1);
			Serial.print(" - ");
			Serial.println(ch2) = ;
			Serial.println("Fail Safe ON!!!");          // Print fail_safe status
#endif // DEBUG
			delay_ms(50);                         // Delay
		}
	}
	LedWrite(0);
}

void drive() {
	static uint16_t dynamic_max = 1900;
	static uint16_t dynamic_min = 1200;
	uint32_t lastTime = 0;
	uint8_t lastPwm = 0;
	/**********************************************************************
	Name: drive();

	Description: In this approach, we use channel 1 value (vertical), as the "power" given
	to the motor and the channel 2 (horizontal) as a de-crescent factor applied to only
	one PWM at time (left or right)

	Usage example: drive();

	**********************************************************************/

#ifdef WATCHDOG_TIMER
	wdt_reset();                                              //  I am still alive baby! (Says to Watchdog that everything is ok!)
#endif

	//H-bridge
	if (ch1 > (((ch1_max + ch1_min) / 2) + dead_band)) {          // FORWARD CASE
		MONITOR_PRINTLN("-------> FOWARD ------->");               // Print direction

		if (status == 2) {                            // Ops! Inverting direction.. Lets wait mosfets completely turn off!
			lastTime = milliseconds;
			while (1) {
				motor2PWMLeft(255);

				if (milliseconds - lastTime > 150)break;
			}
			motor2Stop();
			MONITOR_PRINTLN("<<<!!<<< BACKWARD TO FOWARD <<<!!<<<");       // Print warning!                               // Wait it
		}
		//motor1PWM(map_channel(ch1, ((ch1_max + ch1_min) / 2), ch1_max, pwm_half, pwm_max)); // Drive motor forward proportionally to receiver channel read!
		motor2PWMRight(map(ch1, ((ch1_min + ch1_max) / 2) + dead_band, ch1_max, 255, 0)); // Drive motor forward proportionally to receiver channel read!

		status = 1;                                 // Forward flag
	}
	else if (ch1 < (((ch1_max + ch1_min) / 2) - dead_band)) {         // BACKWARD CASE
		MONITOR_PRINTLN("<-------- BACKWARD <-------");              // Print direction

		if (status == 1) {                            // Ops! Inverting direction.. Lets wait mosfets completely turn off!
			lastTime = milliseconds;
			while (1) {
				motor2PWMRight(255);
				if (milliseconds - lastTime > 150)break;
			}
			motor2Stop();
			MONITOR_PRINTLN(">>>!!>>> FOWARD TO BACKWARD >>>!!>>>");       // Print warning!                                 // Wait it
		}

		//motor1PWM(map_channel(ch1, ch1_min, ((ch1_max + ch1_min) / 2), pwm_min, pwm_half)); // Drive motor backward proportionally to receiver channel read!
		motor2PWMLeft(map(ch1, ch1_min, ((ch1_min + ch1_max) / 2) - dead_band, 0, 255)); // Drive motor backward proportionally to receiver channel read!
		status = 2;                               // Backward flag
	}
	else {                                                       // STOPPED CASE
		MONITOR_PRINTLN("STOPED");                       // Print direction
		//motor1PWM(pwm_half);                        // Turn motor off!
		motor2Stop();
		status = 0;                               // Stopped flag
	}
}

uint16_t map_channel(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void print_sensors() {
#ifdef SERIAL_MONITOR
	uint16_t sensor1, sensor2, ultrassonic;
	ultrassonic = ultrassonic_getValue();
	Serial.print("Ultras. : ");
	if (ultrassonic != 0) Serial.print(ultrassonic);
	else Serial.print("fault");

	sensor1 = sensor1_getValue();
	Serial.print("  Sensor1 : ");
	if (sensor1 != 0) Serial.print(sensor1);
	else Serial.print("fault");

	sensor2 = sensor2_getValue();
	Serial.print("  Sensor2 : ");
	if (sensor2 != 0) Serial.println(sensor2);
	else Serial.println("fault");
#endif // MONI
}
void robotProcess() {
	//print_sensors();
	//delay_ms(100);
#ifdef WATCHDOG_TIMER
	wdt_reset();                        //  I am still alive baby! (Says to Watchdog that everything is ok!)
#endif
	ch1 = sensor1_getValue();

	if (!CalibrateRead()) {     // If board jumper is connected then enter into calibration Mode!
		calibration();            // Call calibration procedure
	}

#ifdef SERIAL_MONITOR
	Serial.print("Channels :");     // Print channel value
	Serial.print("\tCH1: ");
	Serial.print(ch1);
	Serial.print(" \tCH2: ");
	Serial.println(ch2);
#endif // MONI
	fail_safe();            // Call fail_safe check procedure
	ch1 = sensor1_getValue();
	drive();                // Call drive procedure

#ifdef ENABLE_FAILSAFE

#endif // ENABLE_FAILSAFE
}