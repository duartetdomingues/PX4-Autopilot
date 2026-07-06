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

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uavcan_stm32h7/uavcan_stm32h7.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>

constexpr uint32_t CAN_BITRATE = 1000000;
constexpr uint32_t TX_TIMEOUT_MS = 100;
constexpr uint32_t SDO_TIMEOUT_MS = 200;
constexpr uint32_t CONTROL_PERIOD_US = 50000;
constexpr uint32_t RC_TIMEOUT_US = 500000;
constexpr uint32_t CAN_NODE_TIMEOUT_US = 1000000;
constexpr uint8_t THOMSON_ID_1 = 35;
constexpr uint8_t THOMSON_ID_2 = 36;
constexpr uint8_t MD80_ACC_ID = 10;
constexpr uint8_t MD80_GEAR_ID = 11;

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
constexpr uint16_t MD80_STATUSWORD_INTERNAL_LIMIT = 1U << 11;

constexpr int8_t MD80_MODE_SERVICE = -2;
constexpr int8_t MD80_MODE_IDLE = 0;
constexpr int8_t MD80_MODE_PROFILE_POSITION = 1;
constexpr int8_t MD80_MODE_PROFILE_VELOCITY = 3;
constexpr int8_t MD80_MODE_CYCLIC_SYNC_POSITION = 8;
constexpr int8_t MD80_MODE_CYCLIC_SYNC_VELOCITY = 9;

extern "C" uint16_t board_get_can_interfaces(void);

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

	enum class GearSlot : uint8_t {
		Reverse = 0,
		Neutral,
		First,
		Second,
		Count,
	};

	int handle_gear_command(int argc, char *argv[]);

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
		hrt_abstime last_rx{0};
	};

	struct Md80Status {
		int32_t position{0};
		int32_t velocity{0};
		int16_t torque{0};
		uint16_t status_word{0};
		int8_t mode{0};
		uint32_t motion_status{0};
		bool motion_status_read_pending{false};
		hrt_abstime last_rx{0};
	};

	using CanInitHelper = uavcan_stm32h7::CanInitHelper<32>;

	bool initialize();
	bool initialize_thomson(uint8_t node_id);
	bool initialize_md80(uint8_t node_id, const char *name);
	bool configure_md80_parameters(uint8_t node_id);
	bool configure_md80_tpdos(uint8_t node_id);
	bool configure_md80_rpdos(uint8_t node_id);
	bool switch_md80_to_operation_enabled(uint8_t node_id);

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
	bool configure_md80_parameter(uint8_t node_id, uint16_t index, uint8_t subindex, const T &desired)
	{
		T current{};

		if (!sdo_read(node_id, index, subindex, current)) {
			return false;
		}

		if (std::memcmp(&current, &desired, sizeof(T)) != 0) {
			return sdo_write(node_id, index, subindex, desired);
		}

		return true;
	}

	bool send_nmt(uint8_t command, uint8_t node_id);
	bool send_thomson_position(unsigned index, uint16_t position);
	bool send_thomson_motion_reset(unsigned index);
	bool send_md80_target(int32_t position);
	bool send_md80_target(uint8_t node_id, int32_t position, int32_t &target_position, hrt_abstime &last_tx);
	bool send_md80_target_sdo(uint8_t node_id, int32_t position);
	bool send_md80_shutdown(uint8_t node_id);
	bool read_md80_motion_status(uint8_t node_id, Md80Status &status);
	void md80_read_all_status(uint8_t node_id);
	void md80_read_pid_parameters(uint8_t node_id);
	void md80_set_pid_parameters(uint8_t node_id, float_t p, float_t i, float_t d, float_t i_limit);
	void apply_enabled_control();
	void apply_brake();
	void send_keepalives();
	void load_gear_positions();
	void update_gear_from_rc();
	bool save_gear_position(GearSlot gear, int32_t position);
	bool gear_slot_from_name(const char *name, GearSlot &gear) const;
	const char *gear_slot_name(GearSlot gear) const;
	bool can_node_is_online(hrt_abstime last_rx) const;
	uint32_t can_node_rx_age_ms(hrt_abstime last_rx) const;
	void print_gear_status() const;
	bool rc_link_is_valid() const;
	bool rc_controls_are_valid() const;

	uint8_t _interface_index;
	uint32_t _interface_mask;
	CanInitHelper _can;
	uavcan::ICanIface *_iface{nullptr};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	input_rc_s _input_rc{};

	ThomsonCommand _thomson_command[2]{};
	ThomsonStatus _thomson_status[2]{};
	bool _thomson_motion_reset_pending[2]{};
	bool _thomson_initialized[2]{};
	Md80Status _md80_status{};
	Md80Status _gear_status{};
	bool _md80_initialized{false};
	bool _gear_initialized{false};
	int32_t _md80_target_position{0};
	hrt_abstime _md80_last_tx{0};
	int32_t _gear_positions[static_cast<unsigned>(GearSlot::Count)]{};
	int32_t _gear_target_position{0};
	hrt_abstime _gear_last_tx{0};
	bool _gear_target_active{false};
	bool _gear_shutdown_sent{false};
	bool _gear_rc_synchronized{false};
	GearSlot _gear_rc_slot{GearSlot::Neutral};

	bool _initialized{false};
	bool _enable_active{false};
	bool _brake_applied{false};
	bool _relay_rc_latched{false};
	uint32_t _frames_tx{0};
	uint32_t _frames_rx{0};
	uint32_t _sdo_timeouts{0};
	uint32_t _tx_errors{0};
	hrt_abstime _last_tx_error_log{0};
};
