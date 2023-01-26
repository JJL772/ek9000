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


#ifndef _DEVEL7XXX_H_
#define _DEVEL7XXX_H_

#include <asynPortDriver.h>
#include <asynMotorAxis.h>
#include <asynMotorController.h>
#include <motor.h>
#include <drvModbusAsyn.h>
#include <modbus.h>
#include <modbusInterpose.h>
#include <epicsStdlib.h>

#include "pdo/pdo_el7041.h"
#include "pdo/pdo_el7047.h"

#include "devEK9000.h"

// NOTE: When adding a new term type, you will need to modify the fun branches in el70xxPdoAdapter
enum MotorTerminalType {
	MT_EL7041,
	MT_EL7047,
	MT_COUNT,
};

struct el70xxSavedPdo {

	enum Type {
		IN, OUT
	};

	void* ptr;
	size_t size;
	Type type;

	el70xxSavedPdo(void* buf, size_t size, Type type) {
		this->type = type;
		this->ptr = malloc(size);
		this->size = size;
		memcpy(this->ptr, buf, size);
	}

	~el70xxSavedPdo() {
		free(ptr);
	}
};

/**
 * @brief Adapter for different PDO types
 * Handles save/restore of PDOs, getting at the underlying data and getting sizes of
 * underlying data.
 * This is a POD struct, can be trivially copied or memcpy'ed around if you want.
 * 
 * Some notes:
 * - This class is ugly. Definitely the best option to manage N different
 *   PDO types, but it's extremely manual. You need to provide getters and setters
 *   for all underlying types you wish to access. This could probably be classified
 *   as "bad code"!
 *   Could be fixed by code generation using python, or template hacks. I didn't
 *   try any template hacks here out of fear of C++03 support in this codebase.
 *   Ideally we'd be able to not support C++03 for motor (it's a feature flag anyway)
 *   but I'm playing it safe for now. Don't really feel like rewriting this much later
 *   on if someone asks for C++03...
 * 
*/
class epicsShareClass el70xxPdoAdapter {
public:
	el70xxPdoAdapter(MotorTerminalType type) :
		m_type(type)
	{}

	el70xxPdoAdapter() = delete;

	inline MotorTerminalType type() const { return m_type; }

	// PDO pointer accessors

	inline void* in_pdo() {
		return m_type == MT_EL7047 ? (void*)&in.el7047 : (void*)&in.el7041;
	}
	
	inline void* out_pdo() {
		return m_type == MT_EL7047 ? (void*)&out.el7047 : (void*)&out.el7041;
	}

	// PDO size accessors

	inline size_t in_size() const {
		return m_type == MT_EL7047 ? sizeof(in.el7047) : sizeof(in.el7041);
	}

	inline size_t out_size() const {
		return m_type == MT_EL7047 ? sizeof(out.el7047) : sizeof(out.el7041);
	}

	// Save/restore helpers

	el70xxSavedPdo save_output_pdo() {
		return el70xxSavedPdo(
			out_pdo(), out_size(), el70xxSavedPdo::OUT
		);
	}

	el70xxSavedPdo save_input_pdo() {
		return el70xxSavedPdo(
			in_pdo(), in_size(), el70xxSavedPdo::IN
		);
	}

	void restore_pdo(const el70xxSavedPdo& pdo) {
		if (pdo.type == el70xxSavedPdo::IN) {
			assert(pdo.size == in_size());
			memcpy(in_pdo(), pdo.ptr, in_size());
		}
		else if (pdo.type == el70xxSavedPdo::OUT) {
			assert(pdo.size == out_size());
			memcpy(out_pdo(), pdo.ptr, out_size());
		}
		else {
			assert(!"Should not get here!");
		}
	}

	// Helpers to avoid messy memset calls
	
	inline void clear_out() { memset(out_pdo(), 0, out_size()); }
	inline void clear_in() { memset(in_pdo(), 0, in_size()); }

	// Access as a specific type

	inline S_EL7047_PositionInterface_Input* el7047_in() {
		assert(m_type == MT_EL7047);
		return &in.el7047;
	}
	inline S_EL7041_PositionInterface_Input* el7041_in() {
		assert(m_type == MT_EL7041);
		return &in.el7041;
	}

	inline S_EL7047_PositionInterface_Output* el7047_out() {
		assert(m_type == MT_EL7047);
		return &out.el7047;
	}
	inline S_EL7041_PositionInterface_Output* el7041_out() {
		assert(m_type == MT_EL7041);
		return &out.el7041;
	}

	// Accessors for identical underlying fields (although at different offsets)

	inline bool stm_err() {
		return m_type == MT_EL7041 ?
			in.el7041.stm_err : in.el7047.stm_err;
	}

	inline void set_stm_reset(bool reset) {
		m_type == MT_EL7041 ?
			out.el7041.stm_reset = reset : out.el7047.stm_reset = reset;
	}

	inline void set_pos_execute(bool exec) {
		m_type == MT_EL7041 ?
			out.el7041.pos_execute = exec : out.el7047.pos_execute = exec;
	}

	inline void set_pos_emergency_stop(bool stop) {
		m_type == MT_EL7041 ?
			out.el7041.pos_emergency_stp = stop : out.el7047.pos_emergency_stp = stop;
	}

	inline void set_stm_enable(bool en) {
		m_type == MT_EL7041 ?
			out.el7041.stm_enable = en : out.el7047.stm_enable = en;
	}

	inline void set_pos_start_type(uint32_t type) {
		m_type == MT_EL7041 ?
			out.el7041.pos_start_type = type : out.el7047.pos_start_type;
	}

	inline void set_pos_tgt_pos(uint32_t pos) {
		m_type == MT_EL7041 ?
			out.el7041.pos_tgt_pos = pos : out.el7047.pos_tgt_pos = pos;
	}

	inline void set_pos_accel(uint32_t accel) {
		m_type == MT_EL7041 ?
			out.el7041.pos_accel = accel : out.el7047.pos_accel = accel;
	}

	inline void set_pos_velocity(uint32_t velo) {
		m_type == MT_EL7041 ?
			out.el7041.pos_velocity = velo : out.el7047.pos_velocity = velo;
	}

	inline void set_pos_decel(uint32_t decel) {
		m_type == MT_EL7041 ?
			out.el7041.pos_decel = decel : out.el7047.pos_decel = decel;
	}

	inline uint32_t cntr_val() const {
		return m_type == MT_EL7041 ?
			in.el7041.enc_cnt_val : in.el7047.cntr_val;
	}

	inline bool pos_in_tgt() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.pos_in_target : !!in.el7047.pos_in_tgt;
	}

	inline bool stm_move_pos() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.stm_mov_pos : !!in.el7047.stm_mov_pos;
	}

	inline bool stm_move_neg() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.stm_mov_neg : !!in.el7047.stm_mov_neg;
	}

	// FIXME: No stm_stall in el7041! We're just using err for now.
	inline bool stm_stall() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.stm_err : !!in.el7047.stm_stall;
	}

	inline bool cntr_overflow() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.enc_cntr_overflow : !!in.el7047.cntr_overflow;
	}

	inline bool cntr_underflow() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.enc_cntr_overflow : !!in.el7047.cntr_underflow;
	}

	inline bool pos_err() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.pos_err : !!in.el7047.pos_err;
	}

	inline bool sync_err() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.enc_sync_err : !!in.el7047.sync_err;
	}

	inline bool stm_sync_err() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.stm_sync_err : !!in.el7047.stm_sync_err;
	}

	inline bool stm_warn() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.stm_warn : !!in.el7047.stm_warn;
	}

	inline bool pos_busy() const {
		return m_type == MT_EL7041 ?
			!!in.el7041.pos_busy : !!in.el7047.pos_busy;
	}

	inline void set_enc_set_counter(bool set) {
		m_type == MT_EL7041 ?
			out.el7041.enc_set_counter = set : out.el7047.enc_set_counter = set;
	}

	inline void set_enc_set_counter_val(uint32_t val) {
		m_type == MT_EL7041 ?
			out.el7041.enc_set_counter_val = val : out.el7047.enc_set_counter_val = val;
	}

	inline uint32_t pos_accel() const {
		return m_type == MT_EL7041 ?
			in.el7041.pos_accel : in.el7047.pos_accelerate;
	}


private:
	MotorTerminalType m_type;

	union {
		S_EL7047_PositionInterface_Input el7047;
		S_EL7041_PositionInterface_Input el7041;
	} in;

	union {
		S_EL7047_PositionInterface_Output el7047;
		S_EL7041_PositionInterface_Output el7041;
	} out;

};

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
	el70x7Axis(class el70x7Controller* pC, int axisno);

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
	class el70x7Controller* pC_;
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

#endif //_DEVEL7XXX_H_
