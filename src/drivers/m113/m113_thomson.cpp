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
