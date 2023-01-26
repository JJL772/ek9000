
/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#include <stdint.h>

/* For Positioning interface compact */
/* RxPDOs : 0x1601 0x1602 0x1605 (things written to terminal by epics) */
/* TxPDOs: 0x1A01 0x1A03 0x1A06 (things read from terminal by epics) */
#pragma pack(1)

struct S_EL7047_PositionInterface_Output {
	/* 0x1601 */
	uint32_t enc_enable_lat_c : 1;
	uint32_t enc_enable_lat_epe : 1;
	uint32_t enc_set_counter : 1;
	uint32_t enc_enable_lat_ene : 1;
	uint32_t _r1 : 12;
	uint32_t enc_set_counter_val;

	/* 0x1602 */
	uint32_t stm_enable : 1;
	uint32_t stm_reset : 1;
	uint32_t stm_reduce_torque : 1;
	uint32_t _r2 : 8;
	uint32_t stm_digout1 : 1;
	uint32_t _r3 : 4;

	/* 0x1606 */
	uint32_t pos_execute : 1;
	uint32_t pos_emergency_stp : 1;
	uint32_t _r4 : 14;
	uint32_t pos_tgt_pos;
	uint32_t pos_velocity : 16;
	uint32_t pos_start_type : 16;
	uint32_t pos_accel : 16;
	uint32_t pos_decel : 16;
};

struct S_EL7047_PositionInterface_Input {
	/* 0x1A01, 0x6000 */
	uint32_t latc_valid : 1;	 /* Latch c valid */
	uint32_t latc_ext_valid : 1; /* Latch C extern valid */
	uint32_t cntr_set_done : 1;	 /* The counter was set */
	uint32_t cntr_underflow : 1;
	uint32_t cntr_overflow : 1;
	uint32_t _r1 : 2;
	uint32_t extrap_stall : 1; /* Extrapolated part of the counter was invalid */
	uint32_t stat_inp_a : 1;   /* status of input a */
	uint32_t stat_inp_b : 1;   /* status of inp b */
	uint32_t stat_inp_c : 1;   /* Status of inp c */
	uint32_t _r2 : 1;		   /* 2 bits or one??? */
	uint32_t stat_ext_lat : 1; /* Status of extern latch */
	uint32_t sync_err : 1;	   /* Sync error */
	uint32_t _r3 : 1;
	uint32_t txpdo_toggle : 1; /* Toggled when data is updated */
	uint32_t cntr_val;		   /* The counter value */
	uint32_t lat_val;		   /* Latch value */
	/* 0x1A03, 0x6010 */
	uint32_t stm_rdy_enable : 1; /* Driver stage is ready for enabling */
	uint32_t stm_rdy : 1;		 /* Driver stage is ready for operation */
	uint32_t stm_warn : 1;		 /* warning has happened */
	uint32_t stm_err : 1;		 /* Error has happened */
	uint32_t stm_mov_pos : 1;	 /* Moving in the positive dir */
	uint32_t stm_mov_neg : 1;	 /* Moving in the negative dir */
	uint32_t stm_tor_reduce : 1; /* Reduced torque */
	uint32_t stm_stall : 1;		 /* Motor stall */
	uint32_t _r4 : 3;
	uint32_t stm_sync_err : 1; /* Set if synchronization error in previous step */
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1; /* txpdo toggle  */
	uint32_t _r6 : 1;
	uint32_t stm_txpdo_toggle2 : 1;
	/* 0x1A07 */
	uint32_t pos_busy : 1;
	uint32_t pos_in_tgt : 1;
	uint32_t pos_warn : 1;
	uint32_t pos_err : 1;
	uint32_t pos_calibrated : 1;
	uint32_t pos_accelerate : 1;
	uint32_t pos_decelerate : 1;
	uint32_t _r8 : 9;
	uint32_t pos_actual_pos;
	uint32_t pos_actual_vel : 16;
	uint32_t pos_actual_drive_time;
};

#pragma pack()
