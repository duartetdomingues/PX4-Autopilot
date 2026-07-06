/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 ****************************************************************************/

#include "m113.hpp"
#include "m113_helpers.hpp"

#include <px4_platform_common/log.h>

using m113::put_le;

bool M113::send_md80_target(int32_t position)
{
	return send_md80_target(MD80_ACC_ID, position, _md80_target_position, _md80_last_tx);
}

bool M113::send_md80_target(uint8_t node_id, int32_t position, int32_t &target_position, hrt_abstime &last_tx)
/*

*/
{
	target_position = position;
	uint8_t data[6]{};
	put_le(&data[0], MD80_CONTROLWORD_ENABLE_OPERATION);
	put_le(&data[2], position);
	const bool sent = send_frame(0x400U + node_id, data, sizeof(data));

	if (sent) {
		last_tx = hrt_absolute_time();
	}

	return sent;
}

bool M113::send_md80_target_sdo(uint8_t node_id, int32_t position)
{
	const uint16_t enable_operation = MD80_CONTROLWORD_ENABLE_OPERATION;
	const uint16_t new_setpoint = MD80_CONTROLWORD_ENABLE_OPERATION | (1U << 4);

	// Profile Position starts a move on the rising edge of controlword bit 4.
	return sdo_write(node_id, 0x6040, 0x00, enable_operation)
	       && sdo_write(node_id, 0x607A, 0x00, position)
	       && sdo_write(node_id, 0x6040, 0x00, new_setpoint);
}

bool M113::send_md80_shutdown(uint8_t node_id)
{
	uint8_t data[6]{};
	put_le(&data[0], MD80_CONTROLWORD_SHUTDOWN);
	return send_frame(0x400U + node_id, data, sizeof(data));
}

bool M113::read_md80_motion_status(uint8_t node_id, Md80Status &status)
{
	uint32_t motion_status = 0;
	status.motion_status_read_pending = false;

	if (!sdo_read(node_id, 0x2004, 0x07, motion_status)) {
		PX4_ERR("MD80 %u Motion Status read failed", node_id);
		return false;
	}

	status.motion_status = motion_status;
	PX4_WARN("MD80 %u Motion Status: 0x%08lx", node_id, static_cast<unsigned long>(motion_status));

	if ((motion_status & (1UL << 0)) != 0) {
		PX4_ERR("MD80 %u position is outside configured limits", node_id);
	}

	if ((motion_status & (1UL << 1)) != 0) {
		PX4_ERR("MD80 %u velocity exceeded the configured limit", node_id);
	}

	if ((motion_status & (1UL << 24)) != 0) {
		PX4_WARN("MD80 %u acceleration command was clipped", node_id);
	}

	if ((motion_status & (1UL << 25)) != 0) {
		PX4_WARN("MD80 %u torque command was clipped", node_id);
	}

	if ((motion_status & (1UL << 26)) != 0) {
		PX4_WARN("MD80 %u velocity command was clipped", node_id);
	}

	if ((motion_status & (1UL << 27)) != 0) {
		PX4_WARN("MD80 %u position command was clipped", node_id);
	}

	constexpr uint32_t known_bits = (1UL << 0) | (1UL << 1) | (1UL << 24)
					| (1UL << 25) | (1UL << 26) | (1UL << 27);

	if ((motion_status & ~known_bits) != 0) {
		PX4_WARN("MD80 %u Motion Status contains undocumented bits: 0x%08lx",
			 node_id, static_cast<unsigned long>(motion_status & ~known_bits));
	}

	if (motion_status == 0) {
		PX4_INFO("MD80 %u Motion Status has no active errors or warnings", node_id);
	}

	return true;
}

bool M113::switch_md80_to_operation_enabled(uint8_t node_id)
{
	for (unsigned attempt = 0; attempt < 50 && !should_exit(); ++attempt) {
		uint16_t status_word = 0;

		if (!sdo_read(node_id, 0x6041, 0x00, status_word)) {
			PX4_ERR("MD80 %u status read failed", node_id);
			return false;
		}

		const uint8_t state = status_word & 0xFF;

		if (state == MD80_OPERATION_ENABLED) {
			return true;
		}

		uint16_t control_word = 0;

		switch (state) {
		case MD80_SWITCH_ON_DISABLED:
			control_word = MD80_CONTROLWORD_SHUTDOWN;
			break;

		case MD80_READY_TO_SWITCH_ON:
			control_word = MD80_CONTROLWORD_SWITCH_ON;
			break;

		case MD80_SWITCHED_ON:
			control_word = MD80_CONTROLWORD_ENABLE_OPERATION;
			break;

		default:
			PX4_ERR("MD80 %u unexpected CiA 402 state: 0x%02x", node_id, state);
			return false;
		}

		if (!sdo_write(node_id, 0x6040, 0x00, control_word)) {
			PX4_ERR("MD80 %u control word write failed", node_id);
			return false;
		}

		sleep_servicing(200);
	}

	if (should_exit()) {
		return false;
	}

	PX4_ERR("MD80 %u CiA 402 transition timed out", node_id);
	return false;
}

void M113::md80_read_all_status(uint8_t node_id)
{
    uint32_t st = 0;

    sdo_read(node_id, 0x1001, 0x00, st);
    PX4_WARN("MD80 %u Error Register 0x1001: 0x%08lx", node_id, (unsigned long)st);

    uint16_t system_status_index = 0x2003;

    st = 0;
    sdo_read(node_id, system_status_index, 0x01, st);
    PX4_WARN("MD80 %u Main Encoder Status 0x%04lx:1: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x02, st);
    PX4_WARN("MD80 %u Aux Encoder Status 0x%04lx:2: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x03, st);
    PX4_WARN("MD80 %u Calibration Status 0x%04lx:3: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x04, st);
    PX4_WARN("MD80 %u Bridge Status 0x%04lx:4: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x05, st);
    PX4_WARN("MD80 %u Hardware Status 0x%04lx:5: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x06, st);
    PX4_WARN("MD80 %u Homing Status 0x%04lx:6: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x07, st);
    PX4_WARN("MD80 %u Motion Status 0x%04lx:7: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x08, st);
    PX4_WARN("MD80 %u Communication Status 0x%04lx:8: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);

    st = 0;
    sdo_read(node_id, system_status_index, 0x09, st);
    PX4_WARN("MD80 %u Misc Status 0x%04lx:9: 0x%08lx", node_id, (unsigned long)system_status_index, (unsigned long)st);
}

void M113::md80_read_pid_parameters(uint8_t node_id)
{
    float_t p = 0.0f;
    float_t i = 0.0f;
    float_t d = 0.0f;
    float_t i_limit = 0.0f;

    sdo_read(node_id, 0x2001, 0x01, p);
    sdo_read(node_id, 0x2001, 0x02, i);
    sdo_read(node_id, 0x2001, 0x03, d);
    sdo_read(node_id, 0x2001, 0x04, i_limit);

    PX4_INFO("MD80 %u PID parameters: P=%.3f I=%.3f D=%.3f I_limit=%.3f", node_id, (double)p, (double)i, (double)d, (double)i_limit);
}

void M113::md80_set_pid_parameters(uint8_t node_id, float_t p, float_t i, float_t d, float_t i_limit)
{
    sdo_write(node_id, 0x2001, 0x01, p);
    sdo_write(node_id, 0x2001, 0x02, i);
    sdo_write(node_id, 0x2001, 0x03, d);
    sdo_write(node_id, 0x2001, 0x04, i_limit);

    PX4_INFO("MD80 %u PID parameters set: P=%.3f I=%.3f D=%.3f I_limit=%.3f", node_id, (double)p, (double)i, (double)d, (double)i_limit);
}
