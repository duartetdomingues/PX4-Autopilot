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

#include <px4_platform_common/log.h>

#include <cstdlib>
#include <cstring>

namespace
{

bool parse_thomson_value(const char *text, long minimum, long maximum, long &value)
{
	char *end = nullptr;
	value = std::strtol(text, &end, 10);
	return end != text && *end == '\0' && value >= minimum && value <= maximum;
}

} // namespace

bool M113::initialize_thomson(uint8_t node_id)
{
	if (!send_nmt(0x01, node_id)) {
		return false;
	}

	uint32_t cob_id = 0;

	if (!sdo_read(node_id, 0x1800, 0x01, cob_id, 300)) {
		PX4_WARN("Thomson %u TPDO read failed, node disabled", node_id);
		return false;
	}

	if ((cob_id & 0x80000000UL) != 0) {
		if (!send_nmt(0x80, node_id)) {
			return false;
		}

		sleep_servicing(20);
		cob_id &= 0x7FFFFFFFUL;

		if (!sdo_write(node_id, 0x1800, 0x01, cob_id, 300)) {
			PX4_ERR("Thomson %u TPDO enable failed", node_id);
			return false;
		}

		if (!send_nmt(0x01, node_id)) {
			return false;
		}

		sleep_servicing(20);
	}

	PX4_INFO("Thomson %u initialized", node_id);
	return true;
}

int M113::handle_thomson_command(int argc, char *argv[])
{
	if (argc != 6 || std::strcmp(argv[1], "set") != 0) {
		return print_usage("usage: m113 thomson set 1|2 position current speed");
	}

	long actuator = 0;
	long position = 0;
	long current = 0;
	long speed = 0;

	if (!parse_thomson_value(argv[2], 1, THOMSON_ID_2, actuator)
	    || !parse_thomson_value(argv[3], 0, THOMSON_MAX_POSITION, position)
	    || !parse_thomson_value(argv[4], 0, THOMSON_MAX_CURRENT, current)
	    || !parse_thomson_value(argv[5], 0, THOMSON_MAX_SPEED, speed)) {
		return print_usage("invalid Thomson actuator or setpoint");
	}

	unsigned index = 0;

	if (actuator == 1 || actuator == THOMSON_ID_1) {
		index = 0;

	} else if (actuator == 2 || actuator == THOMSON_ID_2) {
		index = 1;

	} else {
		return print_usage("Thomson actuator must be 1, 2, 35, or 36");
	}

	if (!_thomson_initialized[index]) {
		PX4_ERR("Thomson %u is not initialized", index == 0 ? THOMSON_ID_1 : THOMSON_ID_2);
		return PX4_ERROR;
	}

	ThomsonCommand &command = _thomson_command[index];
	command.current_limit = static_cast<uint16_t>(current);
	command.target_speed = static_cast<uint8_t>(speed);

	if (!send_thomson_position(index, static_cast<uint16_t>(position))) {
		PX4_ERR("Thomson %u command send failed", index == 0 ? THOMSON_ID_1 : THOMSON_ID_2);
		return PX4_ERROR;
	}

	PX4_INFO("Thomson %u target: position=%ld current=%ld speed=%ld",
		 index == 0 ? THOMSON_ID_1 : THOMSON_ID_2, position, current, speed);
	return PX4_OK;
}
