#include "Arduino.h"
#include "PID_v1.h"
#include "Wire.h"
#include "MotorController.h"
#include "string.h"

#define SERIAL_DEBUG true

#define PIN_MOT_LEFT 7
#define PIN_MOT_RIGHT 8
#define PIN_MOT_PWM 11
#define PIN_LED 13
#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 3
const int PIN_POT = A0;
#define PIN_ENDSWITCH 4

#define INTERVAL 10 // ms

unsigned long lastTime;
unsigned long lastTrigger = 0;
long lastTriggerInterval = 0;
bool lowSpeedMode = true;

double maxOutput = 1200;

// speed control
double setSpeedValue = 0;
double speedOutput = 0;
double outVal;
double kp_speed = 0.15;
double ki_speed= 0.7;
double kd_speed = 0.0;

// position control
double setPosition = 0;
// double position = 0;
double lastPosition = 0;
double posOutput = 0;
double kp_pos = 4.0;
double ki_pos = 0.0;
double kd_pos = 0.1;

// homing TODO: use homingSpeed instead of cur max speed
double homingSpeed = -500.0;

struct ControllerData controllerData;

PID speedController(&controllerData.speed, &speedOutput, &setSpeedValue, kp_speed, ki_speed, kd_speed, P_ON_E, DIRECT);
PID posController(&controllerData.position, &posOutput, &setPosition, kp_pos, ki_pos, kd_pos, P_ON_E, DIRECT);


#define TWI_BUFFER_SIZE 10
uint8_t twiBuffer[TWI_BUFFER_SIZE];

#define SERIAL_BUFFER_SIZE 20
char serialReceiveBuffer[SERIAL_BUFFER_SIZE+1];

bool dumpValuesSerial = false;
bool homed = false;

// =========================================================================================

void encoderSignal() {
	if( digitalRead(PIN_ENCODER_B) ) {
		controllerData.position++;
		lastTriggerInterval = millis() - lastTrigger;
	} else {
		controllerData.position--;
		lastTriggerInterval = - long(millis() - lastTrigger);
	}
	lastTrigger = millis();
}

/**
 * Param value: -255 to 255
 * LEFT: value < 0
 * RIGHT: value > 0
 */
void setMotorPwm(double value) {
	if( value < 0 ) {
		digitalWrite(PIN_MOT_LEFT, HIGH);
		digitalWrite(PIN_MOT_RIGHT, LOW);
		analogWrite(PIN_MOT_PWM, -value);
	} else {
		digitalWrite(PIN_MOT_LEFT, LOW);
		digitalWrite(PIN_MOT_RIGHT, HIGH);
		analogWrite(PIN_MOT_PWM, value);
	}
}

// =========================================================================================

void stop() {
	controllerData.clearFlag(STATUS_RUNNING);
	controllerData.clearFlag(STATUS_MOVING);
	controllerData.clearFlag(STATUS_HOMING);
	if( SERIAL_DEBUG )
		Serial.println("STOP");
}

void home( int16_t speed ) {
	if( SERIAL_DEBUG ) {
		Serial.print("start homing at speed: ");
		Serial.println(speed);
	}
	controllerData.setFlag(STATUS_MOVING);
	controllerData.setFlag(STATUS_RUNNING);
	controllerData.setFlag(STATUS_HOMING);

	setMaxSpeed(speed);

	controllerData.position = 0;

	// TODO make max homing distance configurable
	if( speed > 0 )
		setPosition = 2400.0;
	else
		setPosition = -2400.0;
}

void setPosTunings( double kp, double ki, double kd ) {
	kp_pos = kp;
	ki_pos = ki;
	kd_pos = kd;
	posController.SetTunings(kp_pos, ki_pos, kd_pos);

	if( SERIAL_DEBUG ) {
		Serial.print("pos PID params: ");
		Serial.print(kp, 3); Serial.print(", ");
		Serial.print(ki, 3); Serial.print(", ");
		Serial.println(kd, 3);
	}
}

void setSpeedTunings( double kp, double ki, double kd ) {
	kp_speed = kp;
	ki_speed = ki;
	kd_speed = kd;
	speedController.SetTunings(kp_speed, ki_speed, kd_speed);

	if( SERIAL_DEBUG ) {
		Serial.print("speed PID params: ");
		Serial.print(kp, 3); Serial.print(", ");
		Serial.print(ki, 3); Serial.print(", ");
		Serial.println(kd, 3);
	}
}

void setMaxSpeed(double maxOut) {
	maxOutput = constrain(maxOut, 50.0, 1400.0);
	if( SERIAL_DEBUG ) {
		Serial.print("set max output: ");
		Serial.println(maxOutput);
	}
	posController.SetOutputLimits(-maxOutput, maxOutput);
}

void runAtSpeed(double speed) {
	setSpeedValue = speed;
	if( SERIAL_DEBUG ) {
		Serial.print("run at speed: ");
		Serial.println(setSpeedValue);
	}
	controllerData.setFlag(STATUS_RUNNING);
	controllerData.clearFlag(STATUS_MOVING);
	controllerData.clearFlag(STATUS_HOMING);
}

void moveToPos(int pos) {
	if( !homed ) {
		if( SERIAL_DEBUG )
			Serial.println("not homed!");
		return;
	}
	setPosition = pos;

	if( SERIAL_DEBUG ) {
		Serial.print("move to pos: ");
		Serial.println(setPosition);
	}

	controllerData.setFlag(STATUS_MOVING);
	controllerData.clearFlag(STATUS_RUNNING);
	controllerData.clearFlag(STATUS_HOMING);
}

void dumpState() {
	Serial.print("State: pos=");
	Serial.print(controllerData.position);
	Serial.print(" speed=");
	Serial.print(controllerData.speed);

	Serial.println();
}

// =========================================================================================

void twiRequest() {
	// return controller data
	Wire.write((uint8_t*)&controllerData, sizeof(controllerData));
	// Serial.print("### TWI data requested: "); Serial.println(sizeof(controllerData));
}

void twiReceive(int numBytes) {
	if( SERIAL_DEBUG )
		Serial.print("twiReceive: ");

	int index = 0;
	while( index < TWI_BUFFER_SIZE && Wire.available() ) {
		twiBuffer[index] = Wire.read();

		if( SERIAL_DEBUG ) {
			Serial.print(twiBuffer[index]);
			Serial.print(" ");
		}
		index++;
	}

	if( index < 1 ) {
		if( SERIAL_DEBUG )
			Serial.println(" ERROR: received 0 bytes");
		return;
	}
	if( SERIAL_DEBUG )
		Serial.println();

	// parse commands
	switch(twiBuffer[0]) {
	case TWI_CMD_STOP:
		stop();
		break;
	case TWI_CMD_RUN:
		runAtSpeed( *((int16_t*)&twiBuffer[1]) );
		break;
	case TWI_CMD_MOVE_TO:
		moveToPos( *((int32_t*)&twiBuffer[1]) );
		break;
	case TWI_CMD_MOVE_BY:
		moveToPos( *((int32_t*)&twiBuffer[1]) + controllerData.position );
		break;
	case TWI_CMD_HOME:
		home( *((int16_t*)&twiBuffer[1]) );
		break;
	case TWI_CMD_SETSPEED:
		setMaxSpeed( *((int16_t*)&twiBuffer[1]) );
		break;
	default:
		if( SERIAL_DEBUG )
			Serial.println("unknown TWI command!");
	}

}

void serialReceive() {
	if( Serial.available() <= 0 )
		return;

	static int bufferLen = 0;

	int ch = Serial.read();
	while( ch >= 0 ) {
		if( ch == '\r' || ch == '\n' || bufferLen >= SERIAL_BUFFER_SIZE ) {
			Serial.println(serialReceiveBuffer);

			if( strncmp(serialReceiveBuffer, "dumpoff", 7) == 0 ) {
				dumpValuesSerial = false;
			} else if( strncmp(serialReceiveBuffer, "dumpon", 7) == 0 ) {
				dumpValuesSerial = true;
			} else if( strncmp(serialReceiveBuffer, "maxout", 6) == 0 ) {
				setMaxSpeed(atof(serialReceiveBuffer+6));
			} else if( strcmp(serialReceiveBuffer, "state") == 0 ) {
				dumpState();
			} else if( strcmp(serialReceiveBuffer, "stop") == 0 ) {
				stop();
			} else if( strncmp(serialReceiveBuffer, "ps", 2) == 0 ) {
				setSpeedTunings(atof(serialReceiveBuffer+2), ki_speed, kd_speed);
			} else if( strncmp(serialReceiveBuffer, "is", 2) == 0 ) {
				setSpeedTunings( kp_speed, atof(serialReceiveBuffer+2), kd_speed);
			} else if( strncmp(serialReceiveBuffer, "ds", 2) == 0 ) {
				setSpeedTunings( kp_speed, ki_speed, atof(serialReceiveBuffer+2));
			} else if( strncmp(serialReceiveBuffer, "pp", 2) == 0 ) {
				setPosTunings(atof(serialReceiveBuffer+2), ki_pos, kd_pos);
			} else if( strncmp(serialReceiveBuffer, "ip", 2) == 0 ) {
				setPosTunings( kp_pos, atof(serialReceiveBuffer+2), kd_pos);
			} else if( strncmp(serialReceiveBuffer, "dp", 2) == 0 ) {
				setPosTunings( kp_pos, ki_pos, atof(serialReceiveBuffer+2));
			} else if( strncmp(serialReceiveBuffer, "hm", 2) == 0 ) {
				home(150);
			} else if( strncmp(serialReceiveBuffer, "hs", 2) == 0 ) {
				homingSpeed = atof(serialReceiveBuffer+2);
			} else if( strncmp(serialReceiveBuffer, "m", 1) == 0 ) {
				moveToPos( atof(serialReceiveBuffer+1) );
			} else if( serialReceiveBuffer[0] == 'r' ) {
				runAtSpeed(atof(serialReceiveBuffer+1));
			} else {
				Serial.println("unknown command!");
			}

			serialReceiveBuffer[0] = 0;
			bufferLen = 0;
		} else {
			serialReceiveBuffer[bufferLen] = (char)ch;
			bufferLen++;
			serialReceiveBuffer[bufferLen] = '\0';
		}

		ch = Serial.read();
	}
}

void calcSpeed(unsigned long timeChange) {
	if( lowSpeedMode ) {
		if( lastTriggerInterval == 0 )
			controllerData.speed = 0;
		else
			controllerData.speed = 1000.0 / lastTriggerInterval;

		lastTriggerInterval = 0;

		if( abs(controllerData.speed) > 100.0 )
			lowSpeedMode = false;
	} else {
		controllerData.speed = (controllerData.position - lastPosition) * 1000.0 / timeChange;

		if( abs(controllerData.speed) < 70.0 )
			lowSpeedMode = true;
	}
}

// =========================================================================================

void setup()
{
	if( SERIAL_DEBUG ) {
		Serial.begin(38400);
		Serial.println("Motor Controller V1 " __DATE__ " " __TIME__);
	}

	Wire.begin(MOTORCONTROLLER_TWI_ADDRESS);
	Wire.onRequest(twiRequest);
	Wire.onReceive(twiReceive);

	pinMode(PIN_MOT_LEFT, OUTPUT);
	pinMode(PIN_MOT_RIGHT, OUTPUT);
	pinMode(PIN_MOT_PWM, OUTPUT);
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_ENDSWITCH, INPUT_PULLUP);

	// set prescaler for timer2 to 1
	//TCCR2B = (TCCR2B & B11111000) | _BV(CS20);
	TCCR2B = (TCCR2B & B11111000) | _BV(CS22);

	pinMode(PIN_ENCODER_A, INPUT);
	pinMode(PIN_ENCODER_B, INPUT);
	attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderSignal, RISING);

	speedController.SetSampleTime(INTERVAL);
	speedController.SetOutputLimits(-100.0, 100.0);
	speedController.SetMode(AUTOMATIC);
	setSpeedValue = 0.0;

	posController.SetSampleTime(INTERVAL);
	posController.SetOutputLimits(-maxOutput, maxOutput);
	posController.SetMode(AUTOMATIC);

	controllerData.m_controllerStatus = 0;
	lastTime = millis();
}

void speedControl() {
	lastPosition = controllerData.position;

	// IN-params:
	//   controllerData.speed
	//   setSpeedValue
	// OUT-param:
	//   speedOutput
	speedController.Compute();

	outVal = setSpeedValue / 8.0;
//	if( outVal < -5 )
//		outVal -= 32;
//	else if( outVal > 5 )
//		outVal += 32;
//	else
//		outVal = 0.0;
	outVal += speedOutput;
	outVal = constrain(outVal, -255, 255);
	setMotorPwm(outVal);
}

// =========================================================================================

void loop()
{
	if( SERIAL_DEBUG )
		serialReceive();

	if( controllerData.isHoming() ) {
		if( !digitalRead(PIN_ENDSWITCH) ) { // LOW: end switch reached
			stop();
			setSpeedValue = 0.0;
			setMotorPwm(0.0);
			controllerData.position = 0;
			homed = true;

		}
	}

	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	if(timeChange >= INTERVAL )
	{
		lastTime = now;

		calcSpeed(timeChange);

		if ( controllerData.isMoving() ) {
			posController.Compute();

			if( abs(setPosition-controllerData.position) < 3 && abs(controllerData.speed) < 10.0 ) {
				posOutput = 0;
				controllerData.clearFlag(STATUS_MOVING);
				if( dumpValuesSerial ) {
					Serial.print("pos reached ");
					Serial.println(controllerData.position, 0);
				}
			}
			// setMotorPwm(posOutput);
			setSpeedValue = posOutput;
			speedControl();

			if( dumpValuesSerial ) {
				Serial.print(controllerData.position, 0);
				Serial.print("\t v: ");
				Serial.print(controllerData.speed, 3);
				Serial.print("\t posOut:");
				Serial.print(posOutput, 2);
				Serial.print("\t speedOut:");
				Serial.print(speedOutput, 2);
				Serial.print("\t PWM:");
				Serial.print(outVal, 0);
				Serial.print("\t LSM:");
				Serial.print(lowSpeedMode);
				Serial.println();
			}
		} else
			setMotorPwm(0.0);
	}

}
