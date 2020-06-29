/*
 * MotorController.h
 *
 *  Created on: 12.03.2020
 *      Author: zwax
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <sys/types.h>


#define MOTORCONTROLLER_TWI_ADDRESS 0x6D

// status bits
#define STATUS_ERROR       1
#define STATUS_HOMING      2
#define STATUS_MOVING      4
#define STATUS_RUNNING     8


/** Command: STOP
 *  Params: none
 */
#define TWI_CMD_STOP 1

/** Command: MOVE_TO: move to given position
 *  Params: position (int32)
 */
#define TWI_CMD_MOVE_TO 2

/** Command: HOME: start homing
 *  Params: none
 */
#define TWI_CMD_HOME 3

/** Command: MOVE: move relative by given offset
 *  Params: offset (int32)
 */
#define TWI_CMD_MOVE_BY 5

/** Command: RUN: start motor with given speed
 *  Params: speed (int16): positiv values: right, negative values: left
 */
#define TWI_CMD_RUN 6

/** Process data, will be transmitted to I2C master */
class ControllerData {
public:
	inline bool isError() { return m_controllerStatus & STATUS_ERROR; };
	inline bool isHoming() { return m_controllerStatus & STATUS_HOMING; };
	inline bool isMoving() { return m_controllerStatus & STATUS_MOVING; };
	inline bool isRunning() { return m_controllerStatus & STATUS_RUNNING; };

	inline void setFlag(uint8_t flag) { m_controllerStatus |= flag; };
	inline void clearFlag(uint8_t flag) { m_controllerStatus &= ~flag; };

public:
	// bit field for status flags
	uint8_t m_controllerStatus;

	double speed;
	double position;
};

#endif /* MOTORCONTROLLER_H_ */
