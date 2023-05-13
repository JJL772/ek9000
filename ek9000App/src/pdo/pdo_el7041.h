
/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#pragma once

#include <stdint.h>

/* For Positioning interface (NOT compact!!!) */
#pragma pack(1)

/** This matches el7047 exactly, according to docs */
/* 0x1601 0x1602 0x1606 */
struct S_EL7041_PositionInterface_Output {
	/* 0x1601 (6 bytes) */
	uint32_t enc_enable_lat_c : 1;
	uint32_t enc_enable_lat_epe : 1;
	uint32_t enc_set_counter : 1;
	uint32_t enc_enable_lat_ene : 1;
	uint32_t _r1 : 12;
	uint32_t enc_set_counter_val;


	/* 0x1602 (2 bytes) */
	uint32_t stm_enable : 1;
	uint32_t stm_reset : 1;
	uint32_t stm_reduce_torque : 1;
	uint32_t stm_digout1 : 1;
	uint32_t _r2 : 4;
	uint8_t _r3;

	/* 0x1606 (14 bytes) */
	uint32_t pos_execute : 1;
	uint32_t pos_emergency_stp : 1;
	uint32_t _r4 : 14;
	uint32_t pos_tgt_pos;
	uint32_t pos_velocity : 16;
	uint32_t pos_start_type : 16;
	uint32_t pos_accel : 16;
	uint32_t pos_decel : 16;
};

/* 1A01, 1A03, 1A06 */
struct S_EL7041_PositionInterface_Input {
	/* 0x1A01 (10 bytes) */
	uint32_t env_latc_valid : 1;
	uint32_t enc_lat_ext_valid : 1; /* Latch extern valid */
	uint32_t enc_cntr_set_done : 1;	 /* The counter was set */
	uint32_t enc_cntr_underflow : 1;
	uint32_t enc_cntr_overflow : 1;
	uint32_t _r1 : 2;
	uint32_t enc_extrap_stall : 1; // 1 byte
	uint32_t enc_inp_a_stat : 1;
	uint32_t enc_inp_b_stat : 1;
	uint32_t enc_inp_c_stat : 1;
	uint32_t _r2 : 1;
	uint32_t enc_ext_lat_stat : 1;	// Status of extern latch
	uint32_t enc_sync_err : 1;		// Sync error
	uint32_t _r3 : 1;
	uint32_t enc_txpdo_toggle : 1; // 2 byte
	uint32_t enc_cnt_val;
	uint32_t enc_lat_val;

	/* 0x1A03 */
	uint32_t stm_rdy_to_enable : 1;
	uint32_t stm_rdy : 1;
	uint32_t stm_warn : 1;
	uint32_t stm_err : 1;
	uint32_t stm_mov_pos : 1;	// Moving positive
	uint32_t stm_mov_neg : 1;	// Moving negative
	uint32_t stm_torque_reduced : 1;
	uint32_t _r4 : 4;
	uint32_t stm_dig1 : 1;		// Digital input 1
	uint32_t stm_dig2 : 1;		// Digital input 2
	uint32_t stm_sync_err : 1;	// Sync error
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1;

	/* 0x1A06 */
	uint32_t pos_busy : 1;
	uint32_t pos_in_target : 1;
	uint32_t pos_warn : 1;
	uint32_t pos_err : 1;
	uint32_t pos_calib : 1;		// Calibrated
	uint32_t pos_accel : 1;		// Accelerate
	uint32_t pos_decel : 1;		// Decelerate
	uint32_t _r6 : 9;
	uint32_t pos_pos;			// Actual pos
	uint32_t pos_velo : 16;		// Actual velocity
	uint32_t pos_drive_time;	// Actual drive time
	
};

#pragma pack()
