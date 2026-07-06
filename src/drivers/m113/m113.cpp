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
#include "m113_relay.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>

extern "C"
{
__EXPORT int m113_main(int argc, char *argv[]);
}

ModuleBase::Descriptor M113::desc{task_spawn, custom_command, print_usage};

M113::M113(uint8_t interface_index) :
	_interface_index(interface_index),
	_interface_mask(1U << interface_index),
	_can(_interface_mask)
{
}

int M113::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return M113::instantiate(ac, av);
	}, argc, argv);
}

int M113::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("m113",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  6144,
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

M113 *M113::instantiate(int argc, char *argv[])
{
	int interface_number = 1;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			interface_number = std::strtol(myoptarg, nullptr, 10);
			break;

		default:
			return nullptr;
		}
	}

	if (interface_number < 1 || interface_number > 2) {
		PX4_ERR("CAN interface must be 1 or 2");
		return nullptr;
	}

	M113 *instance = new M113(static_cast<uint8_t>(interface_number - 1));

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void M113::run()
{
	if (!set_relay_auto_state(false, true)) {
		PX4_ERR("vehicle relay initialization failed");
		return;
	}

	if (!initialize()) {
		(void)set_relay_auto_state(false);

		if (should_exit()) {
			PX4_INFO("M113 initialization cancelled");

		} else {
			PX4_ERR("M113 initialization failed");
		}

		return;
	}

	while (!should_exit()) {
		drain_rx();

		if (_input_rc_sub.updated()) {
			_input_rc_sub.copy(&_input_rc);
			update_gear_from_rc();
		}

		const bool rc_connected = rc_link_is_valid();
		const bool relay_channel_valid = rc_connected
						 && _input_rc.channel_count >= 7
						 && _input_rc.values[6] != UINT16_MAX;

		if (relay_channel_valid) {
			if (_input_rc.values[6] < 1100) {
				_relay_rc_latched = false;

			} else if (_input_rc.values[6] > 1900) {
				_relay_rc_latched = true;
			}

		} else {
			_relay_rc_latched = false;
		}

		const bool controls_valid = rc_controls_are_valid();
		const bool enable_requested = controls_valid && _input_rc.values[4] > 1500;
		const bool relay_on = set_relay_auto_state(_relay_rc_latched) && relay_output_is_on();
		bool enable = false;

		if (!_control_log_initialized
		    || rc_connected != _control_log_rc_connected
		    || enable_requested != _control_log_enable_requested
		    || relay_on != _control_log_relay_on) {
			const unsigned channel_5 = _input_rc.channel_count >= 5 ? _input_rc.values[4] : UINT16_MAX;
			const unsigned channel_7 = _input_rc.channel_count >= 7 ? _input_rc.values[6] : UINT16_MAX;

			PX4_INFO("RC control: link=%s controls=%s ch5=%u enable_request=%s ch7=%u relay_request=%s relay=%s",
				 rc_connected ? "ok" : "lost",
				 controls_valid ? "valid" : "invalid",
				 channel_5,
				 enable_requested ? "on" : "off",
				 channel_7,
				 _relay_rc_latched ? "on" : "off",
				 relay_on ? "on" : "off");

			if (enable_requested && !relay_on) {
				PX4_WARN("unbrake blocked: channel 5 is ON but vehicle relay is OFF");
			}

			_control_log_initialized = true;
			_control_log_rc_connected = rc_connected;
			_control_log_enable_requested = enable_requested;
			_control_log_relay_on = relay_on;
		}

		if (enable_requested && relay_on) {
			enable = true;

			if (!_enable_active) {
				PX4_INFO("unbrake enabled: Thomson35=%s Thomson36=%s MD80=%s",
					 _thomson_initialized[0] ? "ready" : "disabled",
					 _thomson_initialized[1] ? "ready" : "disabled",
					 _md80_initialized ? "ready" : "disabled");
			}

			apply_enabled_control();

		} else {
			if (!_brake_applied) {
				PX4_WARN("applying M113 brake: control inactive");
				apply_brake();
			}
		}

		_enable_active = enable;
		send_keepalives();
		px4_usleep(CONTROL_PERIOD_US);
	}

	PX4_INFO("M113 stop requested, leaving control loop");
	(void)set_relay_auto_state(false);
	PX4_INFO("M113 task stopped");
}

int M113::print_status()
{
	PX4_INFO("CAN%u @ 1 Mbit/s, initialized: %s", _interface_index + 1, _initialized ? "yes" : "no");
	PX4_INFO("RC link: %s, enable: %s, brake: %s, TX: %lu, RX: %lu, TX errors: %lu, SDO timeouts: %lu",
		 rc_link_is_valid() ? "connected" : "disconnected",
		 _enable_active ? "on" : "off",
		 _brake_applied ? "on" : "off",
		 static_cast<unsigned long>(_frames_tx),
		 static_cast<unsigned long>(_frames_rx),
		 static_cast<unsigned long>(_tx_errors),
		 static_cast<unsigned long>(_sdo_timeouts));
	PX4_INFO("RC channel 7 relay command: %s", _relay_rc_latched ? "ON" : "OFF");
	print_relay_status();
	PX4_INFO("Thomson %u: %s position=%u current=%u errors=0x%02x",
		 THOMSON_ID_1,
		 !_thomson_initialized[0] ? "disabled" : can_node_is_online(_thomson_status[0].last_rx) ? "online" : "offline",
		 _thomson_status[0].measured_position,
		 _thomson_status[0].measured_current,
		 _thomson_status[0].error_flags);
	if (can_node_rx_age_ms(_thomson_status[0].last_rx) != UINT32_MAX) {
		PX4_INFO("Thomson %u last CAN RX age: %lu ms", THOMSON_ID_1,
			 static_cast<unsigned long>(can_node_rx_age_ms(_thomson_status[0].last_rx)));
	}

	PX4_INFO("Thomson %u: %s position=%u current=%u errors=0x%02x",
		 THOMSON_ID_2,
		 !_thomson_initialized[1] ? "disabled" : can_node_is_online(_thomson_status[1].last_rx) ? "online" : "offline",
		 _thomson_status[1].measured_position,
		 _thomson_status[1].measured_current,
		 _thomson_status[1].error_flags);
	if (can_node_rx_age_ms(_thomson_status[1].last_rx) != UINT32_MAX) {
		PX4_INFO("Thomson %u last CAN RX age: %lu ms", THOMSON_ID_2,
			 static_cast<unsigned long>(can_node_rx_age_ms(_thomson_status[1].last_rx)));
	}

	PX4_INFO("MD80 %u: %s position=%ld velocity=%ld torque=%d status=0x%04x mode=%d internal_limit=%s motion_status=0x%08lx",
		 MD80_ACC_ID,
		 !_md80_initialized ? "disabled" : can_node_is_online(_md80_status.last_rx) ? "online" : "offline",
		 static_cast<long>(_md80_status.position),
		 static_cast<long>(_md80_status.velocity),
		 _md80_status.torque,
		 _md80_status.status_word,
		 _md80_status.mode,
		 (_md80_status.status_word & MD80_STATUSWORD_INTERNAL_LIMIT) != 0 ? "yes" : "no",
		 static_cast<unsigned long>(_md80_status.motion_status));
	if (can_node_rx_age_ms(_md80_status.last_rx) != UINT32_MAX) {
		PX4_INFO("MD80 %u last CAN RX age: %lu ms", MD80_ACC_ID,
			 static_cast<unsigned long>(can_node_rx_age_ms(_md80_status.last_rx)));
	}

	print_gear_status();
	return 0;
}

int M113::custom_command(int argc, char *argv[])
{
	if (argc >= 1 && strcmp(argv[0], "thomson") == 0) {
		M113 *instance = ModuleBase::get_instance<M113>(desc);

		if (instance == nullptr) {
			return print_usage("m113 must be running for Thomson commands");
		}

		return instance->handle_thomson_command(argc, argv);
	}

	if (argc >= 1 && strcmp(argv[0], "gear") == 0) {
		M113 *instance = ModuleBase::get_instance<M113>(desc);

		if (instance == nullptr) {
			return print_usage("m113 must be running for gear commands");
		}

		return instance->handle_gear_command(argc, argv);
	}

	if (argc == 2 && strcmp(argv[0], "relay") == 0) {
		RelayOverride mode;

		if (strcmp(argv[1], "on") == 0) {
			PX4_WARN("forcing vehicle relay IO PWM OUT 1 GPIO HIGH");
			mode = RelayOverride::ForceOn;

		} else if (strcmp(argv[1], "off") == 0) {
			mode = RelayOverride::ForceOff;

		} else if (strcmp(argv[1], "auto") == 0) {
			mode = RelayOverride::Auto;

		} else if (strcmp(argv[1], "status") == 0) {
			print_relay_status();
			return PX4_OK;

		} else {
			return print_usage("invalid relay mode");
		}

		if (!set_relay_override(mode)) {
			return PX4_ERROR;
		}

		print_relay_status();
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int M113::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the M113 Thomson and MD80 actuators directly over classic CANopen.
This replaces the external ESP32 I2C-to-CAN bridge.

The module owns the selected CAN interface and cannot run beside the UAVCAN daemon.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("m113", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 2, "CAN interface number", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("restart", "Stop and start the M113 controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("relay on", "Force vehicle relay IO PWM OUT 1 GPIO HIGH");
	PRINT_MODULE_USAGE_COMMAND_DESCR("relay off", "Force vehicle relay IO PWM OUT 1 GPIO LOW");
	PRINT_MODULE_USAGE_COMMAND_DESCR("relay auto", "Restore automatic vehicle relay control");
	PRINT_MODULE_USAGE_COMMAND_DESCR("relay status", "Print vehicle relay state");
	PRINT_MODULE_USAGE_COMMAND_DESCR("thomson set 1|2 position current speed",
					 "Command a Thomson actuator (nodes 35 and 36 are also accepted)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gear status", "Print MD80 gear motor position and saved R/N/1/2 positions");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gear config", "Configure and store MD80 gear PDOs, then start TPDO publishing");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gear off", "Disable the MD80 gear motor so it can be moved by hand");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gear go R|N|1|2", "Move the MD80 gear motor to a saved position");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gear set R|N|1|2 [position]", "Save a gear position, defaulting to current motor position");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int m113_main(int argc, char *argv[])
{
	if (argc >= 2 && strcmp(argv[1], "restart") == 0) {
		PX4_INFO("restarting M113");

		if (ModuleBase::stop_command(M113::desc) != PX4_OK) {
			PX4_ERR("M113 restart aborted: stop failed");
			return PX4_ERROR;
		}

		return ModuleBase::start_command(M113::desc, argc - 1, argv + 1);
	}

	return ModuleBase::main(M113::desc, argc, argv);
}
