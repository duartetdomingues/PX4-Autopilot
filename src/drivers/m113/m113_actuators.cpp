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

using namespace time_literals;
using m113::scale_joystick;

bool M113::initialize()
{
	if ((board_get_can_interfaces() & _interface_mask) == 0) {
		PX4_ERR("CAN%u is not available on this board variant", _interface_index + 1);
		return false;
	}

	(void)uavcan_stm32h7::SystemClock::instance();

	const int result = _can.init(CAN_BITRATE);

	if (result < 0) {
		PX4_ERR("CAN%u initialization failed: %d", _interface_index + 1, result);
		return false;
	}

	_iface = _can.driver.getIface(_interface_index);

	if (_iface == nullptr) {
		PX4_ERR("CAN%u interface unavailable after initialization", _interface_index + 1);
		return false;
	}

	PX4_INFO("CAN%u initialized at 1 Mbit/s", _interface_index + 1);

	_thomson_initialized[0] = initialize_thomson(THOMSON_ID_1);

	if (should_exit()) {
		return false;
	}

	_thomson_initialized[1] = initialize_thomson(THOMSON_ID_2);

	sleep_servicing(1000);

	if (should_exit()) {
		return false;
	}

	const uint16_t soft_setting = 0;

	if (_thomson_initialized[0]) {
		_thomson_initialized[0] = sdo_write(THOMSON_ID_1, 0x2003, 0x00, soft_setting)
					  && sdo_write(THOMSON_ID_1, 0x2004, 0x00, soft_setting);

		if (!_thomson_initialized[0]) {
			PX4_WARN("Thomson %u soft-start configuration failed, disabling node", THOMSON_ID_1);
		}
	}

	if (_thomson_initialized[1]) {
		_thomson_initialized[1] = sdo_write(THOMSON_ID_2, 0x2003, 0x00, soft_setting)
					  && sdo_write(THOMSON_ID_2, 0x2004, 0x00, soft_setting);

		if (!_thomson_initialized[1]) {
			PX4_WARN("Thomson %u soft-start configuration failed, disabling node", THOMSON_ID_2);
		}
	}

	_md80_initialized = initialize_md80(MD80_ACC_ID, "accelerator");

	if (should_exit()) {
		return false;
	}

	_gear_initialized = initialize_md80(MD80_GEAR_ID, "gear");

	if (should_exit()) {
		return false;
	}

	_initialized = true;
	load_thomson_positions();
	load_gear_positions();

	if (_gear_initialized) {
		// Set gear to neutral position on startup
		_gear_target_position = _gear_positions[static_cast<unsigned>(GearSlot::Neutral)];
		_gear_target_active = true;
		_gear_shutdown_sent = false;
		_gear_last_tx = 0;

		if (!send_md80_target(MD80_GEAR_ID, _gear_target_position, _gear_target_position, _gear_last_tx)) {
			PX4_WARN("MD80 gear neutral startup target send failed");

		} else {
			PX4_INFO("MD80 gear startup target N: %ld", static_cast<long>(_gear_target_position));
		}
	}

	apply_brake();
	PX4_INFO("M113 direct CANopen controller initialized");
	return true;
}

void M113::apply_enabled_control()
{
	const int joystick_1 = scale_joystick(_input_rc.values[0]);
	const int joystick_2 = scale_joystick(_input_rc.values[2]);
	uint16_t thomson_1_position = _thomson_default_positions[0];
	uint16_t thomson_2_position = _thomson_default_positions[1];

	if (joystick_1 > 510) {
		const float alpha = static_cast<float>(joystick_1 - 500) / 500.0f;
		thomson_1_position = static_cast<uint16_t>((1.0f - alpha) * _thomson_default_positions[0]
				     + alpha * _thomson_brake_positions[0]);
	}

	if (joystick_1 < 490) {
		const float alpha = static_cast<float>(joystick_1) / 500.0f;
		thomson_2_position = static_cast<uint16_t>(alpha * _thomson_default_positions[1]
				     + (1.0f - alpha) * _thomson_brake_positions[1]);
	}

	int32_t md80_position = MD80_LOW_THROTTLE_POSITION;

	if (joystick_2 > 510) {
		const float alpha = static_cast<float>(joystick_2 - 500) / 500.0f;
		md80_position += static_cast<int32_t>(alpha * (MD80_FULL_THROTTLE_POSITION - MD80_LOW_THROTTLE_POSITION));
	}

	(void)send_thomson_position(0, thomson_1_position);
	(void)send_thomson_position(1, thomson_2_position);

	if (_md80_initialized) {
		(void)send_md80_target(md80_position);
	}

	_brake_applied = false;
}

void M113::apply_brake()
{
	(void)send_thomson_position(0, _thomson_brake_positions[0]);
	(void)send_thomson_position(1, _thomson_brake_positions[1]);

	if (_md80_initialized) {
		(void)send_md80_target(0);
	}

	_brake_applied = true;
}

void M113::send_keepalives()
{
	const hrt_abstime now = hrt_absolute_time();

	if (_md80_status.motion_status_read_pending) {
		(void)read_md80_motion_status(MD80_ACC_ID, _md80_status);
	}

	if (_gear_status.motion_status_read_pending) {
		(void)read_md80_motion_status(MD80_GEAR_ID, _gear_status);
	}

	for (unsigned i = 0; i < 2; ++i) {
		if (!_thomson_initialized[i]) {
			continue;
		}

		if (_thomson_motion_reset_pending[i]) {
			(void)send_thomson_motion_reset(i);
			_thomson_motion_reset_pending[i] = false;

		} else if (now - _thomson_command[i].last_tx >= 100_ms) {
			(void)send_thomson_position(i, _thomson_command[i].target_position);
		}
	}

	if (_md80_initialized && now - _md80_last_tx >= 1_s) {
		(void)send_md80_target(_md80_target_position);
	}

	if (!_gear_initialized) {
		return;
	}

	if (_gear_target_active) {
		if (now - _gear_last_tx >= 1_s) {
			(void)send_md80_target(MD80_GEAR_ID, _gear_target_position, _gear_target_position, _gear_last_tx);
		}

		_gear_shutdown_sent = false;

	} else if (!_gear_shutdown_sent) {
		(void)send_md80_shutdown(MD80_GEAR_ID);
		_gear_shutdown_sent = true;
	}
}

bool M113::rc_link_is_valid() const
{
	const hrt_abstime now = hrt_absolute_time();
	return _input_rc.timestamp_last_signal != 0
	       && now - _input_rc.timestamp_last_signal <= RC_TIMEOUT_US
	       && !_input_rc.rc_lost
	       && !_input_rc.rc_failsafe;
}

bool M113::rc_controls_are_valid() const
{
	return rc_link_is_valid()
	       && _input_rc.channel_count >= 5
	       && _input_rc.values[0] != UINT16_MAX
	       && _input_rc.values[2] != UINT16_MAX
	       && _input_rc.values[4] != UINT16_MAX;
}
