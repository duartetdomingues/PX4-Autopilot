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
#include <px4_platform_common/posix.h>

#include <cstring>

using namespace time_literals;
using m113::get_le32;
using m113::put_le;

void M113::pump_driver()
{
	uavcan::CanSelectMasks masks;
	masks.read = _interface_mask;
	const uavcan::CanFrame *pending_tx[uavcan::MaxCanIfaces]{};
	uavcan::ICanDriver &driver = _can.driver;
	(void)driver.select(masks, pending_tx, uavcan_stm32h7::clock::getMonotonic());
}

bool M113::send_frame(uint32_t identifier, const uint8_t *data, uint8_t length)
{
	if (_iface == nullptr || length > uavcan::CanFrame::MaxDataLen) {
		return false;
	}

	uavcan::CanFrame frame {};
	frame.id = identifier;
	frame.dlc = length;
	memcpy(frame.data, data, length);
	const hrt_abstime timeout = hrt_absolute_time() + TX_TIMEOUT_MS * 1000ULL;

	while (!should_exit() && hrt_absolute_time() < timeout) {
		pump_driver();
		const int result = _iface->send(frame,
					       uavcan_stm32h7::clock::getMonotonic() + uavcan::MonotonicDuration::fromMSec(TX_TIMEOUT_MS),
					       0);

		if (result == 1) {
			++_frames_tx;
			return true;
		}

		if (result < 0) {
			break;
		}

		px4_usleep(1000);
	}

	++_tx_errors;
	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_tx_error_log > 1_s) {
		PX4_ERR("CAN%u transmit failed for 0x%03lx", _interface_index + 1, static_cast<unsigned long>(identifier));
		_last_tx_error_log = now;
	}

	return false;
}

void M113::drain_rx()
{
	if (_iface == nullptr) {
		return;
	}

	pump_driver();

	for (unsigned i = 0; i < 64 && !should_exit(); ++i) {
		uavcan::CanFrame frame;
		uavcan::MonotonicTime monotonic_timestamp;
		uavcan::UtcTime utc_timestamp;
		uavcan::CanIOFlags flags = 0;
		const int result = _iface->receive(frame, monotonic_timestamp, utc_timestamp, flags);

		if (result <= 0) {
			break;
		}

		++_frames_rx;
		handle_frame(frame);
	}
}

void M113::handle_frame(const uavcan::CanFrame &frame)
{
	for (unsigned i = 0; i < 2; ++i) {
		const uint8_t node_id = (i == 0) ? THOMSON_ID_1 : THOMSON_ID_2;

		if (frame.id == (0x180U + node_id) && frame.dlc >= 8) {
			ThomsonStatus &status = _thomson_status[i];
			const uint8_t old_error_flags = status.error_flags;
			status.last_rx = hrt_absolute_time();
			status.measured_position = static_cast<uint16_t>(frame.data[0]) | (static_cast<uint16_t>(frame.data[1]) << 8);
			status.measured_current = static_cast<uint16_t>(frame.data[2]) | (static_cast<uint16_t>(frame.data[3]) << 8);
			status.measured_speed = frame.data[4];
			status.measured_load = frame.data[5];
			status.motion_flags = frame.data[6];
			status.error_flags = frame.data[7];

			if (status.error_flags != 0 && status.error_flags != old_error_flags) {
				PX4_ERR("Thomson %u error flags: 0x%02x", node_id, status.error_flags);

				if ((status.error_flags & (1U << 5)) != 0) {
					_thomson_motion_reset_pending[i] = true;
				}
			}

			return;
		}
	}

	Md80Status *md80_status = nullptr;
	uint8_t md80_node_id = 0;

	if ((frame.id & 0x7FU) == MD80_ACC_ID) {
		md80_status = &_md80_status;
		md80_node_id = MD80_ACC_ID;

	} else if ((frame.id & 0x7FU) == MD80_GEAR_ID) {
		md80_status = &_gear_status;
		md80_node_id = MD80_GEAR_ID;
	}

	if (md80_status == nullptr) {
		return;
	}

	if (frame.id == (0x180U + md80_node_id) && frame.dlc >= 2) {
		const uint16_t old_status_word = md80_status->status_word;
		md80_status->last_rx = hrt_absolute_time();
		md80_status->status_word = static_cast<uint16_t>(frame.data[0]) | (static_cast<uint16_t>(frame.data[1]) << 8);

		if ((md80_status->status_word & MD80_STATUSWORD_INTERNAL_LIMIT) != 0
		    && (old_status_word & MD80_STATUSWORD_INTERNAL_LIMIT) == 0) {
			PX4_WARN("MD80 %u internal limit was activated; read 0x2004:07 for details (statusword: 0x%04x)", md80_node_id, md80_status->status_word);
			md80_status->motion_status_read_pending = true;
		}

		if (frame.dlc >= 3) {
			md80_status->mode = static_cast<int8_t>(frame.data[2]);
		}

	} else if (frame.id == (0x280U + md80_node_id) && frame.dlc >= 6) {
		md80_status->last_rx = hrt_absolute_time();
		md80_status->position = static_cast<int32_t>(get_le32(&frame.data[2]));

	} else if (frame.id == (0x380U + md80_node_id) && frame.dlc >= 6) {
		md80_status->last_rx = hrt_absolute_time();
		md80_status->velocity = static_cast<int32_t>(get_le32(&frame.data[2]));

	} else if (frame.id == (0x480U + md80_node_id) && frame.dlc >= 4) {
		md80_status->last_rx = hrt_absolute_time();
		md80_status->torque = static_cast<int16_t>(static_cast<uint16_t>(frame.data[2])
				      | (static_cast<uint16_t>(frame.data[3]) << 8));
	}
}

void M113::sleep_servicing(uint32_t duration_ms)
{
	const hrt_abstime deadline = hrt_absolute_time() + duration_ms * 1000ULL;

	while (!should_exit() && hrt_absolute_time() < deadline) {
		drain_rx();
		px4_usleep(10000);
	}
}

bool M113::wait_sdo_response(uint8_t node_id, uint16_t index, uint8_t subindex,
			     uint8_t response[8], uint32_t timeout_ms)
{
	const hrt_abstime deadline = hrt_absolute_time() + timeout_ms * 1000ULL;

	while (!should_exit() && hrt_absolute_time() < deadline) {
		pump_driver();

		for (unsigned i = 0; i < 64 && !should_exit(); ++i) {
			uavcan::CanFrame frame;
			uavcan::MonotonicTime monotonic_timestamp;
			uavcan::UtcTime utc_timestamp;
			uavcan::CanIOFlags flags = 0;
			const int result = _iface->receive(frame, monotonic_timestamp, utc_timestamp, flags);

			if (result <= 0) {
				break;
			}

			++_frames_rx;

			if (frame.id == (0x580U + node_id) && frame.dlc >= 8
			    && frame.data[1] == static_cast<uint8_t>(index)
			    && frame.data[2] == static_cast<uint8_t>(index >> 8)
			    && frame.data[3] == subindex) {
				std::memcpy(response, frame.data, 8);
				return true;
			}

			handle_frame(frame);
		}

		px4_usleep(1000);
	}

	if (should_exit()) {
		return false;
	}

	++_sdo_timeouts;
	PX4_WARN("SDO timeout node=%u index=0x%04x:%02x", node_id, index, subindex);
	return false;
}

bool M113::sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex,
                    void *value, size_t size, uint32_t timeout_ms)
{
    if (size == 0 || size > 4) {
        PX4_ERR("unsupported SDO read size: %u", static_cast<unsigned>(size));
        return false;
    }

    drain_rx();

    uint8_t request[8]{};
    request[0] = 0x40;
    request[1] = static_cast<uint8_t>(index);
    request[2] = static_cast<uint8_t>(index >> 8);
    request[3] = subindex;

    if (!send_frame(0x600U + node_id, request, sizeof(request))) {
        return false;
    }

    uint8_t response[8]{};

    if (!wait_sdo_response(node_id, index, subindex, response, timeout_ms)) {
        return false;
    }

    if (response[0] == 0x80) {
        PX4_ERR("SDO abort node=%u index=0x%04x:%02x code=0x%08lx",
            node_id, index, subindex,
            static_cast<unsigned long>(get_le32(&response[4])));
        return false;
    }

    const bool expedited = (response[0] & 0x02) != 0;
    const bool size_indicated = (response[0] & 0x01) != 0;
    const size_t response_size = 4 - ((response[0] >> 2) & 0x03);

    if (!expedited || !size_indicated || response_size == 0 || response_size > size) {
        PX4_ERR("unsupported SDO read response node=%u index=0x%04x:%02x cs=0x%02x response_size=%u requested_size=%u",
            node_id, index, subindex, response[0],
            static_cast<unsigned>(response_size),
            static_cast<unsigned>(size));
        return false;
    }

    std::memset(value, 0, size);
    std::memcpy(value, &response[4], response_size);

    return true;
}

bool M113::sdo_write(uint8_t node_id, uint16_t index, uint8_t subindex, const void *value, size_t size,
		     uint32_t timeout_ms)
{
	uint8_t command = 0;

	switch (size) {
	case 1:
		command = 0x2F;
		break;

	case 2:
		command = 0x2B;
		break;

	case 4:
		command = 0x23;
		break;

	default:
		PX4_ERR("unsupported SDO write size: %u", static_cast<unsigned>(size));
		return false;
	}

	drain_rx();
	uint8_t request[8]{};
	request[0] = command;
	request[1] = static_cast<uint8_t>(index);
	request[2] = static_cast<uint8_t>(index >> 8);
	request[3] = subindex;
	std::memcpy(&request[4], value, size);

	if (!send_frame(0x600U + node_id, request, sizeof(request))) {
		return false;
	}

	uint8_t response[8]{};

	if (!wait_sdo_response(node_id, index, subindex, response, timeout_ms)) {
		return false;
	}

	if (response[0] == 0x80) {
		PX4_ERR("SDO abort node=%u index=0x%04x:%02x code=0x%08lx",
			node_id, index, subindex, static_cast<unsigned long>(get_le32(&response[4])));
		return false;
	}

	if (response[0] != 0x60) {
		PX4_ERR("invalid SDO write response node=%u index=0x%04x:%02x cs=0x%02x",
			node_id, index, subindex, response[0]);
		return false;
	}

	return true;
}

bool M113::send_nmt(uint8_t command, uint8_t node_id)
{
	const uint8_t data[2] {command, node_id};
	return send_frame(0x000, data, sizeof(data));
}

bool M113::send_thomson_position(unsigned index, uint16_t position)
{
	if (index >= 2) {
		return false;
	}

	if (!_thomson_initialized[index]) {
		return false;
	}

	if (position > THOMSON_MAX_POSITION) {
		position = THOMSON_MAX_POSITION;
	}

	ThomsonCommand &command = _thomson_command[index];
	command.target_position = position;
	command.motion_enabled = true;
	uint8_t data[8]{};
	put_le(&data[0], command.target_position);
	put_le(&data[2], command.current_limit);
	data[4] = command.target_speed;
	data[5] = command.load_limit;
	data[7] = command.motion_enabled ? 1 : 0;
	const uint8_t node_id = (index == 0) ? THOMSON_ID_1 : THOMSON_ID_2;
	const bool sent = send_frame(0x200U + node_id, data, sizeof(data));

	if (sent) {
		command.last_tx = hrt_absolute_time();
	}

	return sent;
}

bool M113::send_thomson_motion_reset(unsigned index)
{
	if (index >= 2) {
		return false;
	}

	if (!_thomson_initialized[index]) {
		return false;
	}

	ThomsonCommand &command = _thomson_command[index];
	uint8_t data[8]{};
	put_le(&data[0], command.target_position);
	put_le(&data[2], command.current_limit);
	data[4] = command.target_speed;
	data[5] = command.load_limit;
	const uint8_t node_id = (index == 0) ? THOMSON_ID_1 : THOMSON_ID_2;
	data[7] = 0;
	bool ok = send_frame(0x200U + node_id, data, sizeof(data));
	sleep_servicing(20);
	data[7] = 1;
	ok &= send_frame(0x200U + node_id, data, sizeof(data));
	command.motion_enabled = true;
	command.last_tx = hrt_absolute_time();
	return ok;
}
