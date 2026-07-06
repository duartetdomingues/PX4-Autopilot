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

namespace
{

bool parse_thomson_value(const char *text, long minimum, long maximum, long &value)
{
	char *end = nullptr;
	value = std::strtol(text, &end, 10);
	return end != text && *end == '\0' && value >= minimum && value <= maximum;
}

const char *thomson_param_name(unsigned index, M113::ThomsonPositionSlot slot)
{
	if (index >= 2) {
		return nullptr;
	}

	switch (slot) {
	case M113::ThomsonPositionSlot::Brake:
		return index == 0 ? "M113_TH1_BRK" : "M113_TH2_BRK";

	case M113::ThomsonPositionSlot::Default:
		return index == 0 ? "M113_TH1_DEF" : "M113_TH2_DEF";

	case M113::ThomsonPositionSlot::Max:
		return index == 0 ? "M113_TH1_MAX" : "M113_TH2_MAX";

	case M113::ThomsonPositionSlot::Count:
		break;
	}

	return nullptr;
}

} // namespace

void M113::load_thomson_positions()
{
	for (unsigned i = 0; i < 2; ++i) {
		for (unsigned slot_index = 0; slot_index < static_cast<unsigned>(ThomsonPositionSlot::Count); ++slot_index) {
			const auto slot = static_cast<ThomsonPositionSlot>(slot_index);
			const param_t handle = param_find(thomson_param_name(i, slot));
			int32_t position = 0;

			if (handle == PARAM_INVALID || param_get(handle, &position) != PX4_OK) {
				continue;
			}

			if (position < 0) {
				position = 0;

			} else if (position > THOMSON_MAX_POSITION) {
				position = THOMSON_MAX_POSITION;
			}

			switch (slot) {
			case ThomsonPositionSlot::Brake:
				_thomson_brake_positions[i] = static_cast<uint16_t>(position);
				break;

			case ThomsonPositionSlot::Default:
				_thomson_default_positions[i] = static_cast<uint16_t>(position);
				break;

			case ThomsonPositionSlot::Max:
				_thomson_max_positions[i] = static_cast<uint16_t>(position);
				break;

			case ThomsonPositionSlot::Count:
				break;
			}
		}
	}
}

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
	if (argc >= 2 && (std::strcmp(argv[1], "status") == 0 || (argc == 2 && std::strcmp(argv[1], "pos") == 0))) {
		PX4_INFO("Thomson %u positions: brake=%u default=%u max=%u",
			 THOMSON_ID_1, _thomson_brake_positions[0], _thomson_default_positions[0], _thomson_max_positions[0]);
		PX4_INFO("Thomson %u positions: brake=%u default=%u max=%u",
			 THOMSON_ID_2, _thomson_brake_positions[1], _thomson_default_positions[1], _thomson_max_positions[1]);
		return PX4_OK;
	}

	if (argc >= 2 && std::strcmp(argv[1], "pos") == 0) {
		if (argc != 4 && argc != 5) {
			return print_usage("usage: m113 thomson pos 1|2 brake|default|max [position]");
		}

		long actuator = 0;

		if (!parse_thomson_value(argv[2], 1, THOMSON_ID_2, actuator)) {
			return print_usage("invalid Thomson actuator");
		}

		unsigned index = 0;

		if (actuator == 1 || actuator == THOMSON_ID_1) {
			index = 0;

		} else if (actuator == 2 || actuator == THOMSON_ID_2) {
			index = 1;

		} else {
			return print_usage("Thomson actuator must be 1, 2, 35, or 36");
		}

		ThomsonPositionSlot slot{};

		if (!thomson_slot_from_name(argv[3], slot)) {
			return print_usage("Thomson position must be brake, default, or max");
		}

		long position = _thomson_status[index].measured_position;

		if (argc == 5 && !parse_thomson_value(argv[4], 0, THOMSON_MAX_POSITION, position)) {
			return print_usage("invalid Thomson position");
		}

		if (!save_thomson_position(index, slot, static_cast<uint16_t>(position))) {
			return PX4_ERROR;
		}

		PX4_INFO("Thomson %u %s position saved at %ld",
			 index == 0 ? THOMSON_ID_1 : THOMSON_ID_2, thomson_slot_name(slot), position);
		return PX4_OK;
	}

	if (argc != 6 || std::strcmp(argv[1], "set") != 0) {
		return print_usage("usage: m113 thomson set 1|2 position current speed");
	}

	long actuator = 0;
	long position = 0;
	long current = 0;
	long speed = 0;

	if (!parse_thomson_value(argv[2], 1, THOMSON_ID_2, actuator)
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

	if (!parse_thomson_value(argv[3], 0, _thomson_max_positions[index], position)) {
		return print_usage("invalid Thomson position");
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

bool M113::save_thomson_position(unsigned index, ThomsonPositionSlot slot, uint16_t position)
{
	if (index >= 2 || slot == ThomsonPositionSlot::Count || position > THOMSON_MAX_POSITION) {
		return false;
	}

	const param_t handle = param_find(thomson_param_name(index, slot));

	if (handle == PARAM_INVALID) {
		PX4_ERR("Thomson parameter not found");
		return false;
	}

	const int32_t param_value = position;

	if (param_set(handle, &param_value) != PX4_OK) {
		PX4_ERR("Thomson parameter update failed");
		return false;
	}

	if (param_save_default(true) != PX4_OK) {
		PX4_ERR("Thomson parameter save failed");
		return false;
	}

	switch (slot) {
	case ThomsonPositionSlot::Brake:
		_thomson_brake_positions[index] = position;
		break;

	case ThomsonPositionSlot::Default:
		_thomson_default_positions[index] = position;
		break;

	case ThomsonPositionSlot::Max:
		_thomson_max_positions[index] = position;
		break;

	case ThomsonPositionSlot::Count:
		break;
	}

	return true;
}

bool M113::thomson_slot_from_name(const char *name, ThomsonPositionSlot &slot) const
{
	if (std::strcmp(name, "brake") == 0 || std::strcmp(name, "b") == 0) {
		slot = ThomsonPositionSlot::Brake;
		return true;
	}

	if (std::strcmp(name, "default") == 0 || std::strcmp(name, "def") == 0 || std::strcmp(name, "d") == 0) {
		slot = ThomsonPositionSlot::Default;
		return true;
	}

	if (std::strcmp(name, "max") == 0 || std::strcmp(name, "m") == 0) {
		slot = ThomsonPositionSlot::Max;
		return true;
	}

	return false;
}

const char *M113::thomson_slot_name(ThomsonPositionSlot slot) const
{
	switch (slot) {
	case ThomsonPositionSlot::Brake:
		return "brake";

	case ThomsonPositionSlot::Default:
		return "default";

	case ThomsonPositionSlot::Max:
		return "max";

	case ThomsonPositionSlot::Count:
		break;
	}

	return "?";
}
