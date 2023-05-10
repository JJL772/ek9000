/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */
#include <epicsExport.h>
#include <iocsh.h>
#include <callback.h>
#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

/* From motor record */
#include <asynMotorAxis.h>
#include <asynMotorController.h>

#include <cstddef>
#include <stdint.h>
#include <cstring>
#include <cmath>
#include <vector>

#include "asynDriver.h"
#include "devEK9000.h"
#include "devEL7XXX.h"
#include "errlog.h"
#include "ekDiag.h"
#include "ekUtil.h"

enum {
	EL7047_START_TYPE_ABSOLUTE = 0x1,
	EL7047_START_TYPE_RELATIVE = 0x2
};

#define MOTOR_TRACE() asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "(%s) %s:%u\n", __FILE__, __FUNCTION__, __LINE__)

#define BREAK() /* asm("int3\n\t") */

typedef std::vector<el70x7Controller*> ControllerListT;
ControllerListT controllers;

/*
========================================================

class EL70X7Controller

NOTES:
	- None

========================================================
*/

el70x7Controller::el70x7Controller(devEK9000* dev, devEK9000Terminal* controller, const char* port, int numAxis)
	: asynMotorController(port, numAxis, 0, 0, 0, ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0) {
	pcoupler = dev;
	pcontroller = controller;
	this->paxis = (el70x7Axis**)calloc(sizeof(el70x7Axis*), numAxis);
	for (int i = 0; i < numAxis; i++)
		this->paxis[i] = new el70x7Axis(controller->m_terminalId, this, i);
	startPoller(0.25, 0.25, 0);
	if (!dev->VerifyConnection())
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Unable to connect to device.\n");
}

el70x7Axis* el70x7Controller::getAxis(int num) {
	return (el70x7Axis*)asynMotorController::getAxis(num);
}

el70x7Axis* el70x7Controller::getAxis(asynUser* usr) {
	return (el70x7Axis*)asynMotorController::getAxis(usr);
}

void el70x7Controller::report(FILE* fd, int lvl) {
	if (lvl) {
		fprintf(fd, "el70x7Controller slave=%u\n", this->pcontroller->m_terminalIndex);
		fprintf(fd, "\tek9000_name=%s\n", pcoupler->m_name.data());
		fprintf(fd, "\tterminalno=%u\n", pcontroller->m_terminalIndex);
		fprintf(fd, "\tport=%s\n", this->portName);
		fprintf(fd, "\tnumaxes=%u\n", this->numAxes_);
	}
	asynMotorController::report(fd, lvl);
}

/*
========================================================

class EL70X7Axis

NOTES:
	- The EL7047 takes acceleration in ms which represents
	the time to top speed
	- Speed is set when the class is created because it shouldn't
	change during normal operation. I do not want to be doing
	a ton of CoE I/O when we're doing semi-time critical ops
	- This class is the logical representation of a single
	el7047 or el7037

========================================================
*/

el70x7Axis::el70x7Axis(int type, el70x7Controller* pC, int axisnum) :
	asynMotorAxis(pC, axisnum),
	m_pdo(type)
{
	MOTOR_TRACE();
	uint16_t spd;
	uint16_t tmp;
	pC_ = pC;
	this->pcoupler = pC->pcoupler;
	this->pcontroller = pC->pcontroller;
	this->pdrv = pC->pcoupler->m_driver;
	this->lock();
	/* Set previous params to random values */
	/* Grab initial values */
	int status =
		this->pcoupler->m_driver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, pcontroller->m_outputStart,
											 (uint16_t*)m_pdo.out_pdo(), BYTES_TO_REG_SIZE(pcontroller->m_outputSize));
	if (status)
		goto error;
	status = this->pcoupler->m_driver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, pcontroller->m_inputStart,
												  (uint16_t*)m_pdo.in_pdo(), BYTES_TO_REG_SIZE(pcontroller->m_inputSize));
	/* Read the configured speed */
	spd = 0;
	this->pcoupler->doCoEIO(0, pcontroller->m_terminalIndex, coe::EL704X_SPEED_RANGE_INDEX, 1, &spd,
							coe::EL704X_SPEED_RANGE_SUBINDEX);
	/* Clear everything initially */
	this->curr_param.back_accel = 0.0;
	this->curr_param.forward_accel = 0.0;
	this->curr_param.max_vel = 0.0;
	this->curr_param.min_vel = 0.0;

	/* also set these to zero or else it will be full of junk values that'll get possibly written to the device if
	 * there's an input error */
	m_pdo.clear_in();
	m_pdo.clear_out();

	switch (spd) {
		case 0:
			speed = 1000;
			break;
		case 1:
			speed = 2000;
			break;
		case 2:
			speed = 4000;
			break;
		case 3:
			speed = 8000;
			break;
		case 4:
			speed = 16000;
			break;
		case 5:
			speed = 32000;
			break;
		default:
			speed = 1000;
			break;
	}

	m_pdo.set_stm_enable(true); /* Enable the motor */

	/* Default of absolute start type */
	m_pdo.set_pos_start_type(EL7047_START_TYPE_ABSOLUTE);

	/* For acceleration, decel and velocity we want to grab the initial values from the CoE parameters */
	tmp = 0;
	/* Read default min velocity */
	this->pcoupler->doCoEIO(0, pcontroller->m_terminalIndex, coe::EL704X_VELOCITY_MIN_INDEX, 1, &tmp,
							coe::EL704X_VELOCITY_MIN_SUBINDEX);
	m_pdo.set_pos_velocity(tmp);

	/* Read default acceleration in the positive direction */
	this->pcoupler->doCoEIO(0, pcontroller->m_terminalIndex, coe::EL704X_ACCELERATION_POS_INDEX, 1, &tmp,
							coe::EL704X_ACCELERATION_POS_SUBINDEX);
	m_pdo.set_pos_accel(tmp);

	/* Read default deceleration in the positive direction */
	this->pcoupler->doCoEIO(0, pcontroller->m_terminalIndex, coe::EL704X_DECELERATION_POS_INDEX, 1, &tmp,
							coe::EL704X_DECELERATION_POS_SUBINDEX);
	m_pdo.set_pos_decel(tmp);

	this->UpdatePDO();
	this->unlock();
	return;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Modbus IO error. Error=%u\n", __FUNCTION__, __LINE__, status);
	this->unlock();
}

void el70x7Axis::lock() {
	MOTOR_TRACE();
	this->pcoupler->Lock();
}

void el70x7Axis::unlock() {
	MOTOR_TRACE();
	this->pcoupler->Unlock();
}

void el70x7Axis::ResetIfRequired() {
	if (m_pdo.stm_err())
		m_pdo.set_stm_reset(true);
}

asynStatus el70x7Axis::setMotorParameters(uint16_t min_start_vel, uint16_t max_coil_current,
										  uint16_t reduced_coil_currrent, uint16_t nominal_voltage,
										  uint16_t internal_resistance, uint16_t full_steps, uint16_t enc_inc) {
	this->lock();
	MOTOR_TRACE();
	uint16_t tid = pcontroller->m_terminalIndex;
	int stat = pcoupler->doCoEIO(1, tid, coe::EL704X_MAXIMAL_CURRENT_INDEX, 1, &max_coil_current,
								 coe::EL704X_MAXIMAL_CURRENT_SUBINDEX);
	if (stat)
		goto error;
	stat = pcoupler->doCoEIO(1, tid, coe::EL704X_REDUCED_CURRENT_INDEX, 1, &reduced_coil_currrent,
							 coe::EL704X_REDUCED_CURRENT_SUBINDEX);
	if (stat)
		goto error;
	stat = pcoupler->doCoEIO(1, tid, coe::EL704X_NOMINAL_VOLTAGE_INDEX, 1, &nominal_voltage,
							 coe::EL704X_NOMINAL_VOLTAGE_SUBINDEX);
	if (stat)
		goto error;
	stat = pcoupler->doCoEIO(1, tid, coe::EL704X_MOTOR_COIL_RESISTANCE_INDEX, 1, &internal_resistance,
							 coe::EL704X_MOTOR_COIL_RESISTANCE_SUBINDEX);
	if (stat)
		goto error;
	stat = pcoupler->doCoEIO(1, tid, coe::EL704X_MOTOR_FULLSTEPS_INDEX, 1, &full_steps,
							 coe::EL704X_MOTOR_FULLSTEPS_SUBINDEX);
	if (stat)
		goto error;
	stat = pcoupler->doCoEIO(1, tid, coe::EL704X_ENCODER_INCREMENTS_INDEX, 1, &enc_inc,
							 coe::EL704X_ENCODER_INCREMENTS_SUBINDEX);
	if (stat)
		goto error;
	stat = pcoupler->doCoEIO(1, tid, coe::EL704X_START_VELOCITY_INDEX, 1, &min_start_vel,
							 coe::EL704X_START_VELOCITY_SUBINDEX);
	if (stat)
		goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to propagate CoE params error=%u\n", __FUNCTION__,
			  __LINE__, stat);
	this->unlock();
	return asynError;
}

/*
--------------------------------------------------
Move the motor to the specified relative or absolute
position.
rel is set to 0 for absolute position, set to 1 for
a relative pos
min_vel is the starting velo of the motor [steps/s]
max_vel is the actual move velo of the motor [steps/s]
accel is the acceleration of the motor [steps/s^2]
--------------------------------------------------
*/
asynStatus el70x7Axis::move(double pos, int rel, double min_vel, double max_vel, double accel) {
	this->lock();
	MOTOR_TRACE();
	this->ResetIfRequired();

	el70xxSavedPdo prevOut = m_pdo.save_output_pdo();

	/* Set the params */
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "Max vel: %f\n", max_vel);
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "Min vel: %f\n", min_vel);

	/* Update the velocity params */
	m_pdo.set_pos_accel((uint32_t)round(accel));
	m_pdo.set_pos_decel((uint32_t)round(accel));
	m_pdo.set_pos_velocity(min_vel + (max_vel - min_vel) / 2.0f);
	m_pdo.set_pos_tgt_pos(pos);
	if (rel)
		m_pdo.set_pos_start_type(EL7047_START_TYPE_RELATIVE);
	else
		m_pdo.set_pos_start_type(EL7047_START_TYPE_ABSOLUTE);
	m_pdo.set_pos_emergency_stop(false);

	/* Execute move */
	int stat = Execute();
	if (stat)
		goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to perform move.\n", __FUNCTION__, __LINE__);
	m_pdo.restore_pdo(prevOut); /* Restore previous output state on motor error */
	this->unlock();
	return asynError;
}

/*
Move the motor at a constant velocity until the stop signal is recieved.
min_vel is the starting velocity of the motor, max_vel is the max velocity of the motor
the velocity is in steps/s
accel is the acceleration of the motor in steps/s^2
*/
asynStatus el70x7Axis::moveVelocity(double min_vel, double max_vel, double accel) {
	this->lock();
	MOTOR_TRACE();
	this->ResetIfRequired();

	el70xxSavedPdo prevOut = m_pdo.save_output_pdo();

	/* Update relevant parameters */
	m_pdo.set_pos_accel((uint32_t)round(accel));
	m_pdo.set_pos_velocity(min_vel + (max_vel - min_vel) / 2.0);

	int stat = this->Execute();
	if (stat)
		goto error;

	/* Set velocity params */
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to set move velocity.\n", __FUNCTION__, __LINE__);
	m_pdo.restore_pdo(prevOut); /* Restore the pre-call output state on propagation failure */
	this->unlock();
	return asynError;
}

/*
Move the motor to it's home position
*/
asynStatus el70x7Axis::home(double min_vel, double max_vel, double accel, int forwards) {
	// FIXME: we are not taking min vel or forwards into account
	UNUSED(min_vel);
	UNUSED(forwards);

	this->lock();
	MOTOR_TRACE();
	this->ResetIfRequired();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "Motor home\n");
	m_pdo.set_pos_accel((uint32_t)round(accel));
	m_pdo.set_pos_velocity((uint32_t)round(max_vel));

	/* Home is just going to be 0 for now */
	m_pdo.set_pos_tgt_pos(0);
	m_pdo.set_pos_emergency_stop(false);
	m_pdo.set_pos_start_type(EL7047_START_TYPE_ABSOLUTE);
	int stat = Execute();
	if (stat)
		goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to go to home position.\n", __FUNCTION__, __LINE__);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::stop(double accel) {
	this->lock();
	MOTOR_TRACE();
	this->ResetIfRequired();

	/* Update the deceleration field */
	m_pdo.set_pos_decel((uint32_t)round(accel));
	m_pdo.set_pos_execute(false);

	int stat = this->UpdatePDO();
	if (stat)
		goto error;
	setIntegerParam(pC_->motorStop_, 1);
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to stop motor.\n", __FUNCTION__, __LINE__);
	this->unlock();
	return asynError;
}

/*
--------------------------------------------------
This function should poll the motor controller.
it will read the motor position, drive status,
moving status and home status and set each param
using setDoubleParam or something similar.
--------------------------------------------------
*/
asynStatus el70x7Axis::poll(bool* moving) {
	static bool is_polling_thread_ready = false;

	if (!is_polling_thread_ready) {
		is_polling_thread_ready = true;
		return asynSuccess;
	}

	if (!this->pcoupler->VerifyConnection()) {
		asynPrint(this->pasynUser_, ASYN_TRACE_WARNING, "%s:%u Polling skipped because device is not connected.\n",
				  __FUNCTION__, __LINE__);
		return asynSuccess;
	}

	this->lock();
	MOTOR_TRACE();
	/* This will read params from the motor controller */
	int stat = this->UpdatePDO();
	if (stat)
		goto error;

	/* encoderposition is double */
	this->setDoubleParam(pC_->motorEncoderPosition_, (double)m_pdo.cntr_val());
	this->setIntegerParam(pC_->motorStatusDone_, m_pdo.pos_in_tgt());
	this->setIntegerParam(pC_->motorStatusDirection_, m_pdo.stm_move_pos());
	this->setIntegerParam(pC_->motorStatusSlip_, m_pdo.stm_stall());
	this->setIntegerParam(pC_->motorStatusProblem_, m_pdo.stm_err());

	/* Check for counter overflow or underflow */
	if (m_pdo.cntr_overflow() || m_pdo.cntr_underflow())
		asynPrint(this->pasynUser_, ASYN_TRACE_WARNING, "%s: Stepper motor counter overflow/underflow detected.\n",
				  __FUNCTION__);
	/* Checo for other error condition */
	if (m_pdo.stm_err() || m_pdo.pos_err() || m_pdo.sync_err() || m_pdo.stm_sync_err())
		asynPrint(this->pasynUser_, ASYN_TRACE_WARNING, "%s: Stepper motor error detected.\n", __FUNCTION__);
	if (m_pdo.stm_warn())
		asynPrint(this->pasynUser_, ASYN_TRACE_WARNING, "%s: Stepper motor warning.\n", __FUNCTION__);
	if (moving)
		*moving = m_pdo.pos_busy() != 0;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to poll device.\n", __FUNCTION__, __LINE__);
	/* When we reconnect we want to stop the moder ASAP */
	m_pdo.set_pos_emergency_stop(true);
	m_pdo.set_pos_execute(false);
	this->unlock();
	return asynError;
}

/*
--------------------------------------------------
This method should set the position of the motor,
but it should not actually move the motor.
The position is an absolute position that should
be set in the hardware.
--------------------------------------------------
*/
asynStatus el70x7Axis::setPosition(double pos) {
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::setPosition val=%f\n", __FUNCTION__, __LINE__, pos);

	/* Save previous output state */
	el70xxSavedPdo prevOut = m_pdo.save_output_pdo();

	m_pdo.set_pos_tgt_pos((uint32_t)round(pos));
	m_pdo.set_pos_start_type(EL7047_START_TYPE_ABSOLUTE);
	m_pdo.set_pos_decel(m_pdo.pos_accel());
	int stat = this->UpdatePDO();
	if (stat)
		goto error;

	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Error while setting tgt pos.\n", __FUNCTION__, __LINE__);
	/* Restore previous if setPos failed */
	m_pdo.restore_pdo(prevOut);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::setEncoderPosition(double pos) {
	this->lock();
	MOTOR_TRACE();

	m_pdo.set_enc_set_counter_val((uint32_t)round(pos));
	m_pdo.set_enc_set_counter(true);
	int stat = UpdatePDO();
	if (stat)
		goto error;

	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Error while setting encoder position.\n", __FUNCTION__,
			  __LINE__);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::setClosedLoop(bool closed) {
	UNUSED(closed);
	return asynSuccess;
}

asynStatus el70x7Axis::UpdatePDO(bool locked) {
	UNUSED(locked);
	const char* pStep = "UPDATE_PDO";

	el70xxSavedPdo prevInput = m_pdo.save_input_pdo();

	/* Update input pdos */
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n", __FUNCTION__, __LINE__, pStep);
	int stat = pcoupler->m_driver->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, pcontroller->m_inputStart,
											  (uint16_t*)m_pdo.in_pdo(), STRUCT_REGISTER_SIZE(m_pdo.in_size()));
	if (stat)
		goto error;
	pStep = "PROPAGATE_PDO";
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n", __FUNCTION__, __LINE__, pStep);
	/* Propagate changes from our internal pdo */
	stat = pcoupler->m_driver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, pcontroller->m_outputStart,
										  (uint16_t*)m_pdo.out_pdo(), STRUCT_REGISTER_SIZE(m_pdo.out_size()));
	if (stat)
		goto error;
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR,
			  "%s:%u Error while updating PDOs. Error=%u Step=%s Input Start=%u"
			  " Input size=%u Output Start=%u Output Size=%u\n",
			  __FUNCTION__, __LINE__, stat, pStep, pcontroller->m_inputStart, pcontroller->m_inputSize / 2,
			  pcontroller->m_outputStart, pcontroller->m_outputSize / 2);
	/* Reset to previous on error */
	m_pdo.restore_pdo(prevInput);
	// this->output = old_output;
	return asynError;
}

/*
Clears the execute bit and returns after a short sleep
*/
void el70x7Axis::ResetExec() {
	m_pdo.set_pos_execute(false);
	UpdatePDO();
	epicsThreadSleep(0.05); /* 500 uS sleep */
}

/*
Executes a move by setting the execute bit and propagating changes
*/
asynStatus el70x7Axis::Execute(bool locked) {
	UNUSED(locked);

	MOTOR_TRACE();
	this->ResetExec();
	m_pdo.set_pos_execute(true);
	m_pdo.set_pos_emergency_stop(false);
	int stat = this->UpdatePDO();
	if (stat)
		goto error;
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to execute moves.\n", __FUNCTION__, __LINE__);
	return asynError;
}

void el70x7Axis::report(FILE* fd, int lvl) {
	if (lvl) {
		fprintf(fd, "asynMotorAxis");
		fprintf(fd, "\tport=%s\n", this->pC_->portName);
		fprintf(fd, "\tterminal=%u\n", this->pcontroller->m_terminalIndex);
	}
	asynMotorAxis::report(fd, lvl);
}

/*
 * ek7047Configure(string ek9k, )
 */
void el7047_Configure(const iocshArgBuf* args) {
	const char* ek9k = args[0].sval;
	const char* port = args[1].sval;
	const char* rec = args[2].sval;
	int slaveid = args[3].ival;
	const char* type = args[4].sval;
	if (!ek9k) {
		epicsPrintf("Please provide an ek9000 name.\n");
		return;
	}
	if (!port || !rec) {
		epicsPrintf("Please provide a port name.\n");
		return;
	}
	if (!type) {
		epicsPrintf("Please provide a terminal type code (e.g. EL7047)");
		return;
	}

	int termType = -1;
	if (!epicsStrCaseCmp("EL7047", type) || !epicsStrCaseCmp("7047", type))
		termType = 7047;
	else if (!epicsStrCaseCmp("EL7041", type) || !epicsStrCaseCmp("7041", type))
		termType = 7041;

	if (termType <= 0) {
		epicsPrintf("Invalid terminal type '%s'!\n", type);
		return;
	}

	devEK9000* dev = devEK9000::FindDevice(ek9k);

	if (!dev) {
		epicsPrintf("Device not found.\n");
		return;
	}
	
	const STerminalInfoConst_t* info = NULL;
	switch(termType) {
	case 7047:
		info = &EL7047_Info; break;
	case 7041:
		info = &EL7041_Info; break;
	default:
		epicsPrintf("Invalid terminal type '%s'!\n", type);
		break;
	}
	
	devEK9000Terminal* term = &dev->m_terms[slaveid - 1];
	dev->AddTerminal(rec, termType, slaveid);
	term->m_inputSize = info->m_nInputSize;
	term->m_outputSize = info->m_nOutputSize;
	el70x7Controller* pctl = new el70x7Controller(dev, term, port, 1);
	epicsPrintf("Created motor port %s\n", port);
	controllers.push_back(pctl);
}

void el7047_Stat(const iocshArgBuf* args) {
	const char* ek9k = args[0].sval;
	if (!ek9k) {
		epicsPrintf("Please provide an ek9000 name.\n");
		return;
	}
	devEK9000* dev = devEK9000::FindDevice(ek9k);
	if (!dev) {
		epicsPrintf("Invalid device.\n");
		return;
	}
	for (ControllerListT::iterator x = controllers.begin(); x != controllers.end(); ++x) {
		for (int i = 0;; i++) {
			el70x7Axis* axis = (*x)->getAxis(i);
			if (!axis)
				break;
			epicsPrintf("%s\n", (*x)->pcontroller->m_recordName.data());
			epicsPrintf("\tSpeed [steps/s]:      %u\n", axis->speed);
			epicsPrintf("\tEncoder pos:          %u\n", axis->enc_pos);
		}
	}
}

/* Read from CoE object */
void el70x7ReadCoE(const iocshArgBuf* args) {
	const char* ek9k = args[0].sval;
	const char* port = args[1].sval;
	int index = args[2].ival;
	int subindex = args[3].ival;
	int len = args[4].ival;

	if (!ek9k || !port || index < 0 || index > 65535 || subindex < 0 || subindex > 65535 || len < 0)
		return;

	for (ControllerListT::iterator it = controllers.begin(); it != controllers.end(); ++it) {
		el70x7Controller* x = *it;
		if (strcmp(port, x->portName) == 0) {
			x->pcoupler->Lock();
			uint16_t data[32];
			x->pcoupler->doCoEIO(0, x->pcontroller->m_terminalIndex, index, len, data, subindex);
			for (int j = 0; j < len; j++)
				epicsPrintf("%u ", data[j]);
			epicsPrintf("\n");
			x->pcoupler->Unlock();
			return;
		}
	}
	epicsPrintf("Port not found.\n");
}

struct diag_info_t {
	const char* name;
	uint16_t index;
	uint16_t subindex;
};

// clang-format off
static diag_info_t diag_info[] = {
	{"saturated", 0xA010, 0x1},
	{"over-temp", 0xA010, 0x2},
	{"torque-overload", 0xA010, 0x3},
	{"under-voltage", 0xA010, 0x4},
	{"over-voltage", 0xA010, 0x5},
	{"short", 0xA010, 0x6},
	{"no-control-pwr", 0xA010, 0x8},
	{"misc-err", 0xA010, 0x9},
	{"conf", 0xA010, 0xA},
	{"stall", 0xA010, 0xB},
	{}
};
// clang-format on

void el70x7PrintDiag(const iocshArgBuf* args) {
	const char* port = args[0].sval;
	if (!port) {
		epicsPrintf("No such port.\n");
		return;
	}
	for (ControllerListT::iterator it = controllers.begin(); it != controllers.end(); ++it) {
		el70x7Controller* x = *it;
		if (strcmp(port, x->portName) == 0) {
			x->pcoupler->Lock();
			uint16_t data = 0;
			int i = 0;
			for (diag_info_t info = diag_info[0]; info.name; info = diag_info[++i]) {
				x->pcoupler->doCoEIO(0, x->pcontroller->m_terminalIndex, info.index, 1, &data, info.subindex);
				epicsPrintf("\t%s: %s\n", info.name, data == 0 ? "false" : "true");
			}
			x->pcoupler->Unlock();
		}
	}
}

bool iszero(const char* str, int len) {
	for (; len > 0; len--) {
		if (str[len - 1] != '\0')
			return false;
	}
	return true;
}

void el70x7PrintMessages(const iocshArgBuf* args) {
	const char* port = args[0].sval;
	if (!port)
		return;
	for (ControllerListT::iterator it = controllers.begin(); it != controllers.end(); ++it) {
		el70x7Controller* x = *it;
		if (strcmp(x->portName, port) == 0) {
			x->pcoupler->Lock();
			uint16_t string[15];
			x->pcoupler->doCoEIO(0, x->pcontroller->m_terminalIndex, 0x1008, 5, string, 0);
			string[5] = 0;
			for (int i = 0x6; i < 0x38; i++) {
				x->pcoupler->doCoEIO(0, x->pcontroller->m_terminalIndex, 0x10f3, 14, string, i);
				string[14] = 0;
				const char* cstring = (const char*)string;
				if (iszero(cstring, 30))
					continue;
				char strbuf[4096];
				COE_DecodeDiagString(string, strbuf, 4096);
				printf("--------------------------------------------\n");
				printf("#%u %s\n", i - 0x6, strbuf);
				for (int j = 0; j < 30; j++)
					printf("%02X ", (unsigned char)cstring[j]);
				printf("\n");
				printf("---------------------------------------------\n");
			}
			x->pcoupler->Unlock();
		}
	}
}

/* Reset the motor from error state */
void el70x7ResetMotor(const iocshArgBuf* args) {
	const char* port = args[0].sval;
	if (!port)
		return;
	for (ControllerListT::iterator it = controllers.begin(); it != controllers.end(); ++it) {
		el70x7Controller* x = *it;
		if (strcmp(x->portName, port) == 0) {
			/* For this, we need to toggle the reset bit in case it's already been reset once */
			el70x7Axis* axis = x->getAxis(0);
			axis->m_pdo.set_pos_execute(false);
			axis->m_pdo.set_stm_reset(false);
			axis->UpdatePDO();
			axis->m_pdo.set_stm_reset(true);
			axis->UpdatePDO();
		}
	}
}

void el7047_register() {
	/* el7047Configure */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg arg2 = {"Port Name", iocshArgString};
		static const iocshArg arg3 = {"Record", iocshArgString};
		static const iocshArg arg4 = {"Terminal position", iocshArgInt};
		static const iocshArg arg5 = {"Terminal type", iocshArgString};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3, &arg4, &arg5};
		static const iocshFuncDef func = {"el70x7Configure", 5, args,
										  "el70x7Configure ek9k_name port_name record_name term_number terminal_type"};
		iocshRegister(&func, el7047_Configure);
	}
	/* el70x7Stat */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"el70x7Stat", 1, args, "el70x7Stat ek9k_name"};
		iocshRegister(&func, el7047_Stat);
	}
	/* el70x7PrintMessages */
	{
		static const iocshArg arg1 = {"Port", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"el70x7PrintMessages", 1, args, "el70x7PrintMessages port_name"};
		iocshRegister(&func, el70x7PrintMessages);
	}
	/* el70x7Reset */
	{
		static const iocshArg arg0 = {"EL70x7 Port Name", iocshArgString};
		static const iocshArg* const args[] = {&arg0};
		static const iocshFuncDef func = {"el70x7Reset", 1, args, "el70x7Reset port_name"};
		iocshRegister(&func, el70x7ResetMotor);
	}
	/* el70x7PrintDiag */
	{
		static const iocshArg arg0 = {"EL70x7 Port Name", iocshArgString};
		static const iocshArg* const args[] = {&arg0};
		static const iocshFuncDef func = {"el70x7PrintDiag", 1, args, "el70x7PrintDiag port_name"};
		iocshRegister(&func, el70x7PrintDiag);
	}
}

extern "C"
{
	epicsExportRegistrar(el7047_register);
}
