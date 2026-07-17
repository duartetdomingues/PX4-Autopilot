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

#include <parameters/param.h>
#include <px4_platform_common/log.h>

#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cmath>

namespace
{

const char *gear_param_name(M113::GearSlot gear)
{
	switch (gear) {
	case M113::GearSlot::Reverse:
		return "M113_GEAR_R";

	case M113::GearSlot::Neutral:
		return "M113_GEAR_N";

	case M113::GearSlot::First:
		return "M113_GEAR_1";

	case M113::GearSlot::Second:
		return "M113_GEAR_2";

	case M113::GearSlot::Count:
		break;
	}

	return nullptr;
}

} // namespace

void M113::load_gear_positions()
{
	for (unsigned i = 0; i < static_cast<unsigned>(GearSlot::Count); ++i) {
		const GearSlot gear = static_cast<GearSlot>(i);
		const param_t handle = param_find(gear_param_name(gear));

		if (handle != PARAM_INVALID) {
			(void)param_get(handle, &_gear_positions[i]);
		}
	}
}

void M113::update_gear_from_rc()
{
	if (!rc_link_is_valid()
	    || _input_rc.channel_count < 6
	    || _input_rc.values[5] == UINT16_MAX) {
		_gear_rc_synchronized = false;
		return;
	}

	const uint16_t channel = _input_rc.values[5];
	GearSlot gear;

	if (channel < 1375) {
		gear = GearSlot::Reverse;

	} else if (channel < 1625) {
		gear = GearSlot::Neutral;

	} else if (channel < 1875) {
		gear = GearSlot::First;

	} else {
		gear = GearSlot::Second;
	}

	if (!_gear_rc_synchronized) {
		_gear_rc_slot = gear;
		_gear_rc_synchronized = true;
		PX4_INFO("RC channel 6 synchronized at gear %s; first value ignored", gear_slot_name(gear));
		return;
	}

	if (gear == _gear_rc_slot) {
		return;
	}

	_gear_rc_slot = gear;

	if (!_gear_initialized) {
		PX4_WARN("RC channel 6 selected gear %s, but MD80 gear is disabled", gear_slot_name(gear));
		return;
	}

	const unsigned index = static_cast<unsigned>(gear);
	_gear_target_position = _gear_positions[index];
	_gear_target_active = true;
	_gear_shutdown_sent = false;
	_gear_last_tx = 0;
	PX4_INFO("RC channel 6 gear %s target: %ld",
		 gear_slot_name(gear), static_cast<long>(_gear_target_position));
}

bool M113::save_gear_position(GearSlot gear, int32_t position)
{
	if (gear == GearSlot::Count) {
		return false;
	}

	const unsigned index = static_cast<unsigned>(gear);
	const param_t handle = param_find(gear_param_name(gear));

	if (handle == PARAM_INVALID) {
		PX4_ERR("gear parameter not found");
		return false;
	}

	if (param_set(handle, &position) != PX4_OK) {
		PX4_ERR("gear parameter update failed");
		return false;
	}

	if (param_save_default(true) != PX4_OK) {
		PX4_ERR("gear parameter save failed");
		return false;
	}

	_gear_positions[index] = position;
	return true;
}

bool M113::gear_slot_from_name(const char *name, GearSlot &gear) const
{
	if (strcmp(name, "R") == 0 || strcmp(name, "r") == 0) {
		gear = GearSlot::Reverse;
		return true;
	}

	if (strcmp(name, "N") == 0 || strcmp(name, "n") == 0) {
		gear = GearSlot::Neutral;
		return true;
	}

	if (strcmp(name, "1") == 0) {
		gear = GearSlot::First;
		return true;
	}

	if (strcmp(name, "2") == 0) {
		gear = GearSlot::Second;
		return true;
	}

	return false;
}

const char *M113::gear_slot_name(GearSlot gear) const
{
	switch (gear) {
	case GearSlot::Reverse:
		return "R";

	case GearSlot::Neutral:
		return "N";

	case GearSlot::First:
		return "1";

	case GearSlot::Second:
		return "2";

	case GearSlot::Count:
		break;
	}

	return "?";
}

bool M113::can_node_is_online(hrt_abstime last_rx) const
{
	const hrt_abstime now = hrt_absolute_time();
	return last_rx != 0 && now - last_rx <= CAN_NODE_TIMEOUT_US;
}

uint32_t M113::can_node_rx_age_ms(hrt_abstime last_rx) const
{
	if (last_rx == 0) {
		return UINT32_MAX;
	}

	return static_cast<uint32_t>((hrt_absolute_time() - last_rx) / 1000ULL);
}

void M113::print_gear_status() const
{
	const uint32_t rx_age_ms = can_node_rx_age_ms(_gear_status.last_rx);
	PX4_INFO("MD80 gear %u: %s last_rx_ms=%lu position=%ld velocity=%ld torque=%d status=0x%04x mode=%d internal_limit=%s motion_status=0x%08lx target=%s",
		 MD80_GEAR_ID,
		 !_gear_initialized ? "disabled" : can_node_is_online(_gear_status.last_rx) ? "online" : "offline",
		 static_cast<unsigned long>(rx_age_ms),
		 static_cast<long>(_gear_status.position),
		 static_cast<long>(_gear_status.velocity),
		 _gear_status.torque,
		 _gear_status.status_word,
		 _gear_status.mode,
		 (_gear_status.status_word & MD80_STATUSWORD_INTERNAL_LIMIT) != 0 ? "yes" : "no",
		 static_cast<unsigned long>(_gear_status.motion_status),
		 _gear_target_active ? "active" : "off");

	PX4_INFO("Gear positions: R=%ld N=%ld 1=%ld 2=%ld",
		 static_cast<long>(_gear_positions[static_cast<unsigned>(GearSlot::Reverse)]),
		 static_cast<long>(_gear_positions[static_cast<unsigned>(GearSlot::Neutral)]),
		 static_cast<long>(_gear_positions[static_cast<unsigned>(GearSlot::First)]),
		 static_cast<long>(_gear_positions[static_cast<unsigned>(GearSlot::Second)]));
}

int M113::handle_gear_command(int argc, char *argv[])
{
	if (argc < 2) {
		return print_usage("missing gear command");
	}

	if (strcmp(argv[1], "status") == 0 || strcmp(argv[1], "pos") == 0) {
		print_gear_status();
		return PX4_OK;
	}

	if (strcmp(argv[1], "off") == 0) {
		_gear_target_active = false;
		PX4_INFO("MD80 gear motor shutdown requested");
		return PX4_OK;
	}

	if (strcmp(argv[1], "pid") == 0) {
		if (argc == 2) {
			md80_read_pid_parameters(MD80_GEAR_ID);
			return PX4_OK;
		}

		if (argc != 7) {
			return print_usage("usage: m113 gear pid position|velocity kp ki kd integral_limit");
		}

		uint16_t index = 0;
		const char *controller = nullptr;

		if (strcmp(argv[2], "position") == 0 || strcmp(argv[2], "pos") == 0) {
			index = 0x2002;
			controller = "position";

		} else if (strcmp(argv[2], "velocity") == 0 || strcmp(argv[2], "vel") == 0) {
			index = 0x2001;
			controller = "velocity";

		} else {
			return print_usage("PID controller must be position or velocity");
		}

		float values[4]{};

		for (unsigned i = 0; i < 4; ++i) {
			char *end = nullptr;
			errno = 0;
			values[i] = std::strtof(argv[i + 3], &end);

			if (errno != 0 || end == argv[i + 3] || *end != '\0' || !std::isfinite(values[i])) {
				return print_usage("PID gains must be finite numbers");
			}
		}

		// 0x1010 Store Parameters is accepted only in the CiA 402
		// "switch on disabled" state. Stop periodic gear commands while the
		// drive is disabled, then restore its operational state afterwards.
		const bool target_was_active = _gear_target_active;
		_gear_target_active = false;
		_gear_shutdown_sent = true;
		const uint16_t disable_voltage = 0;
		const auto resume_gear = [&]() {
			const bool resumed = switch_md80_to_operation_enabled(MD80_GEAR_ID)
					     && sdo_write(MD80_GEAR_ID, 0x6060, 0x00, MD80_MODE_PROFILE_POSITION);
			_gear_target_active = target_was_active;
			_gear_shutdown_sent = false;
			_gear_last_tx = 0;
			return resumed;
		};

		if (!sdo_write(MD80_GEAR_ID, 0x6040, 0x00, disable_voltage)) {
			PX4_ERR("MD80 gear disable voltage failed");
			_gear_target_active = target_was_active;
			_gear_shutdown_sent = false;
			return PX4_ERROR;
		}

		sleep_servicing(200);
		uint16_t status_word = 0;

		if (!sdo_read(MD80_GEAR_ID, 0x6041, 0x00, status_word)
		    || (status_word & 0x004f) != MD80_SWITCH_ON_DISABLED) {
			PX4_ERR("MD80 gear did not enter switch on disabled (status=0x%04x)", status_word);
			(void)resume_gear();
			return PX4_ERROR;
		}

		if (!md80_set_pid_parameters(MD80_GEAR_ID, index, values)) {
			PX4_ERR("MD80 gear %s PID update failed", controller);
			(void)resume_gear();
			return PX4_ERROR;
		}

		const uint32_t save = 0x65766173;

		if (!sdo_write(MD80_GEAR_ID, 0x1010, 0x01, save)) {
			PX4_ERR("MD80 gear %s PID store failed", controller);
			(void)resume_gear();
			return PX4_ERROR;
		}

		sleep_servicing(3000);

		float readback[4]{};

		if (!md80_read_pid_parameters(MD80_GEAR_ID, index, readback)) {
			PX4_ERR("MD80 gear %s PID readback failed", controller);
			(void)resume_gear();
			return PX4_ERROR;
		}

		if (!resume_gear()) {
			PX4_ERR("MD80 gear failed to resume after PID update");
			return PX4_ERROR;
		}

		PX4_INFO("MD80 gear %s PID updated and stored: P=%.6g I=%.6g D=%.6g I_limit=%.6g", controller,
			 (double)readback[0], (double)readback[1], (double)readback[2], (double)readback[3]);
		return PX4_OK;
	}

	if (strcmp(argv[1], "config") == 0) {
		if (!send_nmt(0x82, MD80_GEAR_ID)) {
			PX4_ERR("MD80 gear NMT pre-operational failed");
			return PX4_ERROR;
		}

		sleep_servicing(100);

		if (!sdo_write(MD80_GEAR_ID, 0x6060, 0x00, MD80_MODE_SERVICE)
		    || !configure_md80_tpdos(MD80_GEAR_ID)
		    || !configure_md80_rpdos(MD80_GEAR_ID)) {
			PX4_ERR("MD80 gear PDO configuration failed");
			return PX4_ERROR;
		}

		const uint32_t save = 0x65766173;

		if (!sdo_write(MD80_GEAR_ID, 0x1010, 0x01, save)) {
			PX4_ERR("MD80 gear parameter store failed");
			return PX4_ERROR;
		}

		sleep_servicing(2000);

		if (!sdo_write(MD80_GEAR_ID, 0x6060, 0x00, MD80_MODE_IDLE)
		    || !switch_md80_to_operation_enabled(MD80_GEAR_ID)
		    || !sdo_write(MD80_GEAR_ID, 0x6060, 0x00, MD80_MODE_PROFILE_POSITION)
		    || !send_nmt(0x01, MD80_GEAR_ID)) {
			PX4_ERR("MD80 gear operational start failed");
			return PX4_ERROR;
		}

		_gear_initialized = true;
		_gear_shutdown_sent = false;
		PX4_INFO("MD80 gear PDO configuration stored and started");
		return PX4_OK;
	}

	if (strcmp(argv[1], "go") == 0) {
		if (!_gear_initialized) {
			PX4_ERR("MD80 gear motor is not initialized");
			return PX4_ERROR;
		}

		if (argc != 3) {
			return print_usage("usage: m113 gear go R|N|1|2");
		}

		GearSlot gear{};

		if (!gear_slot_from_name(argv[2], gear)) {
			return print_usage("invalid gear");
		}

		const unsigned index = static_cast<unsigned>(gear);
		_gear_target_position = _gear_positions[index];
		_gear_target_active = true;
		_gear_shutdown_sent = false;
		_gear_last_tx = 0;
		PX4_INFO("MD80 gear target %s: %ld", gear_slot_name(gear), static_cast<long>(_gear_target_position));
		return PX4_OK;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 3 && argc != 4) {
			return print_usage("usage: m113 gear set R|N|1|2 [position]");
		}

		GearSlot gear{};

		if (!gear_slot_from_name(argv[2], gear)) {
			return print_usage("invalid gear");
		}

		const int32_t position = (argc == 4) ? static_cast<int32_t>(std::strtol(argv[3], nullptr, 10)) : _gear_status.position;

		if (!save_gear_position(gear, position)) {
			return PX4_ERROR;
		}

		PX4_INFO("MD80 gear %s saved at %ld", gear_slot_name(gear), static_cast<long>(position));
		return PX4_OK;
	}

	return print_usage("unknown gear command");
}
