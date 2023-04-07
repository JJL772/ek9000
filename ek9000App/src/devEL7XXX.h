/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

/*

Notes about the driver

For EL7047 and EL7037 terminals:
- MUST be configured to use "Position interface compact" PDO type

For EL7041 and EL7031
- MUST be configured to use "Position interface" PDO type (NOT COMPACT)


*/

#pragma once

#include <asynPortDriver.h>
#include <asynMotorAxis.h>
#include <asynMotorController.h>
#include <motor.h>
#include <drvModbusAsyn.h>
#include <modbus.h>
#include <modbusInterpose.h>
#include <epicsStdlib.h>

#include "pdo/adapter.h"
#include "pdo/pdo_el7041.h"
#include "pdo/pdo_el7047.h"

#include "devEK9000.h"

class el70x7Controller;

/*
========================================================

class EL70X7Axis

========================================================
*/
class epicsShareClass el70x7Axis : public asynMotorAxis {
public:
	devEK9000* pcoupler;
	devEK9000Terminal* pcontroller;
	drvModbusAsyn* pdrv;
	el70xxPdoAdapter m_pdo;
	struct {
		double forward_accel;
		double back_accel;
		double max_vel;
		double min_vel;
	} curr_param, prev_param;
	/* This parameter should not be changed during operation of the motor */
	/* FOR NOW AT LEAST */
	uint32_t speed;
	uint32_t enc_pos;

public:
	/*
	terminalType: Terminal type number, comes after the "EL" in the name. Supported values are 7041 and 7047
	pC: Motor controller
	axisno: Axis number, usually will be <= 2 for most EL types
	*/
	el70x7Axis(int terminalType, el70x7Controller* pC, int axisno);

	/*
	min_start_vel: Min starting velocity of the motor (10,000 = 100%)
	max_coil_current: Max current passing thru the motor coils (mA)
	reduced_coil_current: (mA)
	nominal_volage: operating voltage of the motor (mV)
	internal_resistance: internal resistance of the motor (10mOhm)
	full_steps: Number of full motor steps
	enc_inc: the number of increments of the encoder per revolution (4-fold)
	*/
	asynStatus setMotorParameters(uint16_t min_start_vel, uint16_t max_coil_current, uint16_t reduced_coil_currrent,
								  uint16_t nominal_voltage, uint16_t internal_resistance, uint16_t full_steps,
								  uint16_t enc_inc);

	/* Move to home */
	asynStatus move(double pos, int rel, double min_vel, double max_vel, double accel) OVERRIDE;

	/* Move with a velocity */
	asynStatus moveVelocity(double min_vel, double max_vel, double accel) OVERRIDE;

	/* Move to home */
	asynStatus home(double min_vel, double max_vel, double accel, int forwards) OVERRIDE;

	/* Stop with accel */
	asynStatus stop(double accel) OVERRIDE;

	/* Poll */
	asynStatus poll(bool* moving) OVERRIDE;

	/* Set the position target */
	asynStatus setPosition(double pos) OVERRIDE;

	/* Set the position of the encoder in steps from 0 */
	asynStatus setEncoderPosition(double pos) OVERRIDE;

	/* Set if it's closed loop or not */
	asynStatus setClosedLoop(bool closed) OVERRIDE;

	/* Report all detected motor axes */
	void report(FILE* fd, int lvl) OVERRIDE;

	el70xxPdoAdapter& pdo() { return m_pdo; }
	const el70xxPdoAdapter& pdo() const { return m_pdo; }

	/* Locks the driver */
	void lock();

	/* Unlocks the driver */
	void unlock();

public:
	asynStatus UpdatePDO(bool locked = false);
	asynStatus Execute(bool locked = false); /* Execute a move */
	void ResetExec();
	void ResetIfRequired();
	el70x7Controller* pC_;
	friend class el70x7Controller;
};

/*
========================================================

class EL70X7Axis

========================================================
*/

class epicsShareClass el70x7Controller : public asynMotorController {
public:
	devEK9000* pcoupler;
	devEK9000Terminal* pcontroller;
	el70x7Axis** paxis;

public:
	el70x7Controller(devEK9000* dev, devEK9000Terminal* controller, const char* port, int numAxis);

	el70x7Axis* getAxis(int num) OVERRIDE;
	el70x7Axis* getAxis(asynUser* axis) OVERRIDE;

	/* Report all parameters */
	void report(FILE* fd, int lvl) OVERRIDE;

	friend class el70x7Axis;
};

#define EL7047_VELO_MIN_INDEX 13
#define EL7047_VELO_MAX_INDEX 14
#define EL7047_ACCEL_POS_INDEX 15
#define EL7047_ACCEL_NEG_INDEX 16
#define EL7047_DEACCEL_POS_INDEX 17
#define EL7047_DEACCEL_NEG_INDEX 18
#define EL7047_EMERGENCY_DEACCEL_INDEX 19
