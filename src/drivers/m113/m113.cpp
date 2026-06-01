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

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uavcan_stm32h7/uavcan_stm32h7.hpp>

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

using namespace time_literals;

extern "C"
{
__EXPORT int m113_main(int argc, char *argv[]);
uint16_t board_get_can_interfaces(void);
}

namespace
{

constexpr uint32_t CAN_BITRATE = 1000000;
constexpr uint32_t TX_TIMEOUT_MS = 100;
constexpr uint32_t SDO_TIMEOUT_MS = 200;
constexpr uint32_t CONTROL_PERIOD_US = 50000;
constexpr uint32_t RC_TIMEOUT_US = 500000;

constexpr uint8_t THOMSON_ID_1 = 35;
constexpr uint8_t THOMSON_ID_2 = 36;
constexpr uint8_t MD80_ID = 10;

constexpr uint16_t THOMSON_BRAKE_POSITION_1 = 1000;
constexpr uint16_t THOMSON_BRAKE_POSITION_2 = 1000;
constexpr uint16_t THOMSON_DEFAULT_POSITION = 1250;
constexpr uint16_t THOMSON_MAX_POSITION = 1500;
constexpr uint16_t THOMSON_MAX_CURRENT = 150;
constexpr uint8_t THOMSON_MAX_SPEED = 65;

constexpr int32_t MD80_FULL_THROTTLE_POSITION = 16000;
constexpr int32_t MD80_LOW_THROTTLE_POSITION = 10000;

constexpr uint16_t MD80_CONTROLWORD_SHUTDOWN = 6;
constexpr uint16_t MD80_CONTROLWORD_SWITCH_ON = 7;
constexpr uint16_t MD80_CONTROLWORD_ENABLE_OPERATION = 15;

constexpr uint8_t MD80_SWITCH_ON_DISABLED = 0x40;
constexpr uint8_t MD80_READY_TO_SWITCH_ON = 0x21;
constexpr uint8_t MD80_SWITCHED_ON = 0x23;
constexpr uint8_t MD80_OPERATION_ENABLED = 0x27;

constexpr int8_t MD80_MODE_SERVICE = -2;
constexpr int8_t MD80_MODE_IDLE = 0;
constexpr int8_t MD80_MODE_PROFILE_POSITION = 1;

template<typename T>
void put_le(uint8_t *destination, T value)
{
	for (size_t i = 0; i < sizeof(T); ++i) {
		destination[i] = static_cast<uint8_t>(value >> (8 * i));
	}
}

uint32_t get_le32(const uint8_t *source)
{
	return static_cast<uint32_t>(source[0])
	       | (static_cast<uint32_t>(source[1]) << 8)
	       | (static_cast<uint32_t>(source[2]) << 16)
	       | (static_cast<uint32_t>(source[3]) << 24);
}

int scale_joystick(uint16_t raw)
{
	constexpr int min_value = 1102;
	constexpr int max_value = 1927;
	int scaled = (static_cast<int>(raw) - min_value) * 1000 / (max_value - min_value);

	if (scaled < 0) {
		scaled = 0;

	} else if (scaled > 1000) {
		scaled = 1000;
	}

	return scaled;
}

} // namespace

class M113 : public ModuleBase
{
public:
	static Descriptor desc;

	explicit M113(uint8_t interface_index);

	static int task_spawn(int argc, char *argv[]);
	static M113 *instantiate(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	struct ThomsonCommand {
		uint16_t target_position{THOMSON_DEFAULT_POSITION};
		uint16_t current_limit{THOMSON_MAX_CURRENT};
		uint8_t target_speed{THOMSON_MAX_SPEED};
		uint8_t load_limit{0};
		bool motion_enabled{true};
		hrt_abstime last_tx{0};
	};

	struct ThomsonStatus {
		uint16_t measured_position{0};
		uint16_t measured_current{0};
		uint8_t measured_speed{0};
		uint8_t measured_load{0};
		uint8_t motion_flags{0};
		uint8_t error_flags{0};
	};

	struct Md80Status {
		int32_t position{0};
		int32_t velocity{0};
		int16_t torque{0};
		uint16_t status_word{0};
		int8_t mode{0};
	};

	using CanInitHelper = uavcan_stm32h7::CanInitHelper<32>;

	bool initialize();
	bool initialize_thomson(uint8_t node_id);
	bool initialize_md80();
	bool configure_md80_parameters();
	bool configure_md80_tpdos();
	bool configure_md80_rpdos();
	bool switch_md80_to_operation_enabled();

	bool send_frame(uint32_t identifier, const uint8_t *data, uint8_t length);
	void pump_driver();
	void drain_rx();
	void handle_frame(const uavcan::CanFrame &frame);
	void sleep_servicing(uint32_t duration_ms);

	bool wait_sdo_response(uint8_t node_id, uint16_t index, uint8_t subindex,
			       uint8_t response[8], uint32_t timeout_ms);
	bool sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex, void *value, size_t size,
		      uint32_t timeout_ms = SDO_TIMEOUT_MS);
	bool sdo_write(uint8_t node_id, uint16_t index, uint8_t subindex, const void *value, size_t size,
		       uint32_t timeout_ms = SDO_TIMEOUT_MS);

	template<typename T>
	bool sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex, T &value,
		      uint32_t timeout_ms = SDO_TIMEOUT_MS)
	{
		return sdo_read(node_id, index, subindex, &value, sizeof(value), timeout_ms);
	}

	template<typename T>
	bool sdo_write(uint8_t node_id, uint16_t index, uint8_t subindex, const T &value,
		       uint32_t timeout_ms = SDO_TIMEOUT_MS)
	{
		return sdo_write(node_id, index, subindex, &value, sizeof(value), timeout_ms);
	}

	template<typename T>
	bool configure_md80_parameter(uint16_t index, uint8_t subindex, const T &desired)
	{
		T current{};

		if (!sdo_read(MD80_ID, index, subindex, current)) {
			return false;
		}

		if (std::memcmp(&current, &desired, sizeof(T)) != 0) {
			return sdo_write(MD80_ID, index, subindex, desired);
		}

		return true;
	}

	bool send_nmt(uint8_t command, uint8_t node_id);
	bool send_thomson_position(unsigned index, uint16_t position);
	bool send_thomson_motion_reset(unsigned index);
	bool send_md80_target(int32_t position);
	void apply_enabled_control();
	void apply_brake();
	void send_keepalives();
	bool rc_is_valid() const;

	uint8_t _interface_index;
	uint32_t _interface_mask;
	CanInitHelper _can;
	uavcan::ICanIface *_iface{nullptr};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	input_rc_s _input_rc{};

	ThomsonCommand _thomson_command[2]{};
	ThomsonStatus _thomson_status[2]{};
	bool _thomson_motion_reset_pending[2]{};
	Md80Status _md80_status{};
	int32_t _md80_target_position{0};
	hrt_abstime _md80_last_tx{0};

	bool _initialized{false};
	bool _enable_active{false};
	bool _brake_applied{false};
	uint32_t _frames_tx{0};
	uint32_t _frames_rx{0};
	uint32_t _sdo_timeouts{0};
	uint32_t _tx_errors{0};
	hrt_abstime _last_tx_error_log{0};
};

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

	if (!initialize_thomson(THOMSON_ID_1) || !initialize_thomson(THOMSON_ID_2)) {
		return false;
	}

	sleep_servicing(1000);

	uint16_t soft_setting = 0;
	bool thomson_settings_ok = true;
	thomson_settings_ok &= sdo_write(THOMSON_ID_1, 0x2003, 0x00, soft_setting);
	thomson_settings_ok &= sdo_write(THOMSON_ID_1, 0x2004, 0x00, soft_setting);
	thomson_settings_ok &= sdo_write(THOMSON_ID_2, 0x2003, 0x00, soft_setting);
	thomson_settings_ok &= sdo_write(THOMSON_ID_2, 0x2004, 0x00, soft_setting);

	if (!thomson_settings_ok) {
		PX4_ERR("Thomson soft-start configuration failed");
		return false;
	}

	if (!initialize_md80()) {
		return false;
	}

	_initialized = true;
	apply_brake();
	PX4_INFO("M113 direct CANopen controller initialized");
	return true;
}

bool M113::initialize_thomson(uint8_t node_id)
{
	if (!send_nmt(0x01, node_id)) {
		return false;
	}

	uint32_t cob_id = 0;

	if (!sdo_read(node_id, 0x1800, 0x01, cob_id, 300)) {
		PX4_ERR("Thomson %u TPDO read failed", node_id);
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

bool M113::initialize_md80()
{
	if (!configure_md80_parameters()) {
		PX4_ERR("MD80 parameter configuration failed");
		return false;
	}

	const uint32_t save = 0x65766173;

	if (!sdo_write(MD80_ID, 0x1010, 0x01, save)) {
		PX4_ERR("MD80 parameter store failed");
		return false;
	}

	sleep_servicing(2000);

	if (!sdo_write(MD80_ID, 0x6060, 0x00, MD80_MODE_SERVICE)
	    || !configure_md80_tpdos()
	    || !configure_md80_rpdos()) {
		return false;
	}

	const uint8_t set_zero = 1;

	if (!sdo_write(MD80_ID, 0x2003, 0x05, set_zero)) {
		PX4_ERR("MD80 zero-position command failed");
		return false;
	}

	sleep_servicing(100);

	if (!sdo_write(MD80_ID, 0x6060, 0x00, MD80_MODE_IDLE)
	    || !switch_md80_to_operation_enabled()
	    || !sdo_write(MD80_ID, 0x6060, 0x00, MD80_MODE_PROFILE_POSITION)) {
		return false;
	}

	_md80_target_position = 0;
	PX4_INFO("MD80 %u initialized", MD80_ID);
	return true;
}

bool M113::configure_md80_parameters()
{
	bool ok = true;
	const int32_t rated_current = 3800;
	const int32_t rated_torque = 3000;
	const uint16_t max_torque = 3000;
	const uint16_t max_current = 3421;
	const uint32_t max_speed = 233;
	const uint32_t pole_pairs = 14;
	const float torque_constant = 0.120f;
	const uint16_t torque_bandwidth = 500;
	const uint8_t shutdown_temperature = 80;
	const float gear_ratio = 0.166666f;

	ok &= configure_md80_parameter(0x6075, 0x00, rated_current);
	ok &= configure_md80_parameter(0x6076, 0x00, rated_torque);
	ok &= configure_md80_parameter(0x6072, 0x00, max_torque);
	ok &= configure_md80_parameter(0x6073, 0x00, max_current);
	ok &= configure_md80_parameter(0x6080, 0x00, max_speed);
	ok &= configure_md80_parameter(0x2000, 0x01, pole_pairs);
	ok &= configure_md80_parameter(0x2000, 0x02, torque_constant);
	ok &= configure_md80_parameter(0x2000, 0x05, torque_bandwidth);
	ok &= configure_md80_parameter(0x2000, 0x07, shutdown_temperature);
	ok &= configure_md80_parameter(0x2000, 0x08, gear_ratio);
	return ok;
}

bool M113::configure_md80_tpdos()
{
	const uint8_t transmission_type = 0xFE;
	const uint16_t event_timer = 100;

	for (uint8_t pdo = 0; pdo < 4; ++pdo) {
		const uint16_t index = 0x1800 + pdo;
		uint32_t cob_id = (static_cast<uint32_t>(pdo + 1) << 8) + 0x80 + MD80_ID;
		const uint32_t disabled_cob_id = cob_id | 0x80000000UL;

		if (!sdo_write(MD80_ID, index, 0x01, disabled_cob_id)
		    || !sdo_write(MD80_ID, index, 0x02, transmission_type)
		    || !sdo_write(MD80_ID, index, 0x05, event_timer)
		    || !sdo_write(MD80_ID, index, 0x01, cob_id)) {
			PX4_ERR("MD80 TPDO%u configuration failed", pdo + 1);
			return false;
		}

		sleep_servicing(200);
	}

	return true;
}

bool M113::configure_md80_rpdos()
{
	const uint8_t transmission_type = 0xFF;
	const uint32_t rpdo4_disabled = 0x80000500UL + MD80_ID;
	const uint32_t rpdo3_disabled = 0x80000400UL + MD80_ID;
	const uint32_t rpdo3_enabled = 0x00000400UL + MD80_ID;
	const uint32_t rpdo2_disabled = 0x80000300UL + MD80_ID;
	const uint32_t rpdo1_disabled = 0x80000200UL + MD80_ID;

	return sdo_write(MD80_ID, 0x1403, 0x01, rpdo4_disabled)
	       && sdo_write(MD80_ID, 0x1402, 0x01, rpdo3_disabled)
	       && sdo_write(MD80_ID, 0x1402, 0x02, transmission_type)
	       && sdo_write(MD80_ID, 0x1402, 0x01, rpdo3_enabled)
	       && sdo_write(MD80_ID, 0x1401, 0x01, rpdo2_disabled)
	       && sdo_write(MD80_ID, 0x1400, 0x01, rpdo1_disabled);
}

bool M113::switch_md80_to_operation_enabled()
{
	for (unsigned attempt = 0; attempt < 50; ++attempt) {
		uint16_t status_word = 0;

		if (!sdo_read(MD80_ID, 0x6041, 0x00, status_word)) {
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
			PX4_ERR("MD80 unexpected CiA 402 state: 0x%02x", state);
			return false;
		}

		if (!sdo_write(MD80_ID, 0x6040, 0x00, control_word)) {
			return false;
		}

		sleep_servicing(200);
	}

	PX4_ERR("MD80 CiA 402 transition timed out");
	return false;
}

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

	while (hrt_absolute_time() < timeout) {
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

	for (unsigned i = 0; i < 64; ++i) {
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

	if (frame.id == (0x180U + MD80_ID) && frame.dlc >= 2) {
		_md80_status.status_word = static_cast<uint16_t>(frame.data[0]) | (static_cast<uint16_t>(frame.data[1]) << 8);

		if (frame.dlc >= 3) {
			_md80_status.mode = static_cast<int8_t>(frame.data[2]);
		}

	} else if (frame.id == (0x280U + MD80_ID) && frame.dlc >= 6) {
		_md80_status.position = static_cast<int32_t>(get_le32(&frame.data[2]));

	} else if (frame.id == (0x380U + MD80_ID) && frame.dlc >= 6) {
		_md80_status.velocity = static_cast<int32_t>(get_le32(&frame.data[2]));

	} else if (frame.id == (0x480U + MD80_ID) && frame.dlc >= 4) {
		_md80_status.torque = static_cast<int16_t>(static_cast<uint16_t>(frame.data[2])
				      | (static_cast<uint16_t>(frame.data[3]) << 8));
	}
}

void M113::sleep_servicing(uint32_t duration_ms)
{
	const hrt_abstime deadline = hrt_absolute_time() + duration_ms * 1000ULL;

	while (hrt_absolute_time() < deadline) {
		drain_rx();
		px4_usleep(10000);
	}
}

bool M113::wait_sdo_response(uint8_t node_id, uint16_t index, uint8_t subindex,
			     uint8_t response[8], uint32_t timeout_ms)
{
	const hrt_abstime deadline = hrt_absolute_time() + timeout_ms * 1000ULL;

	while (hrt_absolute_time() < deadline) {
		pump_driver();

		for (unsigned i = 0; i < 64; ++i) {
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

	++_sdo_timeouts;
	PX4_ERR("SDO timeout node=%u index=0x%04x:%02x", node_id, index, subindex);
	return false;
}

bool M113::sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex, void *value, size_t size, uint32_t timeout_ms)
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
			node_id, index, subindex, static_cast<unsigned long>(get_le32(&response[4])));
		return false;
	}

	const bool expedited = (response[0] & 0x02) != 0;
	const bool size_indicated = (response[0] & 0x01) != 0;
	const size_t response_size = 4 - ((response[0] >> 2) & 0x03);

	if (!expedited || !size_indicated || response_size != size) {
		PX4_ERR("unsupported SDO read response node=%u index=0x%04x:%02x cs=0x%02x size=%u",
			node_id, index, subindex, response[0], static_cast<unsigned>(response_size));
		return false;
	}

	std::memcpy(value, &response[4], size);
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

bool M113::send_md80_target(int32_t position)
{
	_md80_target_position = position;
	uint8_t data[6]{};
	put_le(&data[0], MD80_CONTROLWORD_ENABLE_OPERATION);
	put_le(&data[2], position);
	const bool sent = send_frame(0x400U + MD80_ID, data, sizeof(data));

	if (sent) {
		_md80_last_tx = hrt_absolute_time();
	}

	return sent;
}

void M113::apply_enabled_control()
{
	const int joystick_1 = scale_joystick(_input_rc.values[0]);
	const int joystick_2 = scale_joystick(_input_rc.values[2]);
	uint16_t thomson_1_position = THOMSON_DEFAULT_POSITION;
	uint16_t thomson_2_position = THOMSON_DEFAULT_POSITION;

	if (joystick_1 > 510) {
		const float alpha = static_cast<float>(joystick_1 - 500) / 500.0f;
		thomson_1_position = static_cast<uint16_t>((1.0f - alpha) * THOMSON_DEFAULT_POSITION
				     + alpha * THOMSON_BRAKE_POSITION_1);
	}

	if (joystick_1 < 490) {
		const float alpha = static_cast<float>(joystick_1) / 500.0f;
		thomson_2_position = static_cast<uint16_t>(alpha * THOMSON_DEFAULT_POSITION
				     + (1.0f - alpha) * THOMSON_BRAKE_POSITION_2);
	}

	int32_t md80_position = MD80_LOW_THROTTLE_POSITION;

	if (joystick_2 > 510) {
		const float alpha = static_cast<float>(joystick_2 - 500) / 500.0f;
		md80_position += static_cast<int32_t>(alpha * (MD80_FULL_THROTTLE_POSITION - MD80_LOW_THROTTLE_POSITION));
	}

	(void)send_thomson_position(0, thomson_1_position);
	(void)send_thomson_position(1, thomson_2_position);
	(void)send_md80_target(md80_position);
	_brake_applied = false;
}

void M113::apply_brake()
{
	(void)send_thomson_position(0, THOMSON_BRAKE_POSITION_1);
	(void)send_thomson_position(1, THOMSON_BRAKE_POSITION_2);
	(void)send_md80_target(0);
	_brake_applied = true;
}

void M113::send_keepalives()
{
	const hrt_abstime now = hrt_absolute_time();

	for (unsigned i = 0; i < 2; ++i) {
		if (_thomson_motion_reset_pending[i]) {
			(void)send_thomson_motion_reset(i);
			_thomson_motion_reset_pending[i] = false;

		} else if (now - _thomson_command[i].last_tx >= 100_ms) {
			(void)send_thomson_position(i, _thomson_command[i].target_position);
		}
	}

	if (now - _md80_last_tx >= 1_s) {
		(void)send_md80_target(_md80_target_position);
	}
}

bool M113::rc_is_valid() const
{
	const hrt_abstime now = hrt_absolute_time();
	return _input_rc.timestamp_last_signal != 0
	       && now - _input_rc.timestamp_last_signal <= RC_TIMEOUT_US
	       && !_input_rc.rc_lost
	       && !_input_rc.rc_failsafe
	       && _input_rc.channel_count >= 5
	       && _input_rc.values[0] != UINT16_MAX
	       && _input_rc.values[2] != UINT16_MAX
	       && _input_rc.values[4] != UINT16_MAX;
}

void M113::run()
{
	if (!initialize()) {
		PX4_ERR("M113 initialization failed");
		return;
	}

	while (!should_exit()) {
		drain_rx();

		if (_input_rc_sub.updated()) {
			_input_rc_sub.copy(&_input_rc);
		}

		const bool enable = rc_is_valid() && _input_rc.values[4] > 1500;

		if (enable) {
			apply_enabled_control();

		} else if (!_brake_applied) {
			PX4_WARN("applying M113 brake: RC enable inactive");
			apply_brake();
		}

		_enable_active = enable;
		send_keepalives();
		px4_usleep(CONTROL_PERIOD_US);
	}

	apply_brake();
	PX4_INFO("M113 stopped with brake applied");
}

int M113::print_status()
{
	PX4_INFO("CAN%u @ 1 Mbit/s, initialized: %s", _interface_index + 1, _initialized ? "yes" : "no");
	PX4_INFO("enable: %s, brake: %s, TX: %lu, RX: %lu, TX errors: %lu, SDO timeouts: %lu",
		 _enable_active ? "on" : "off",
		 _brake_applied ? "on" : "off",
		 static_cast<unsigned long>(_frames_tx),
		 static_cast<unsigned long>(_frames_rx),
		 static_cast<unsigned long>(_tx_errors),
		 static_cast<unsigned long>(_sdo_timeouts));
	PX4_INFO("Thomson %u: position=%u current=%u errors=0x%02x",
		 THOMSON_ID_1, _thomson_status[0].measured_position, _thomson_status[0].measured_current,
		 _thomson_status[0].error_flags);
	PX4_INFO("Thomson %u: position=%u current=%u errors=0x%02x",
		 THOMSON_ID_2, _thomson_status[1].measured_position, _thomson_status[1].measured_current,
		 _thomson_status[1].error_flags);
	PX4_INFO("MD80 %u: position=%ld velocity=%ld torque=%d status=0x%04x mode=%d",
		 MD80_ID,
		 static_cast<long>(_md80_status.position),
		 static_cast<long>(_md80_status.velocity),
		 _md80_status.torque,
		 _md80_status.status_word,
		 _md80_status.mode);
	return 0;
}

int M113::custom_command(int argc, char *argv[])
{
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
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int m113_main(int argc, char *argv[])
{
	return ModuleBase::main(M113::desc, argc, argv);
}
