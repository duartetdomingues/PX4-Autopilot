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

#include "m113_relay.hpp"

#include <drivers/px4io/px4io_driver.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>

namespace
{

px4::atomic<RelayOverride> relay_override{RelayOverride::Auto};
px4::atomic<bool> relay_auto_on{false};
px4::atomic<bool> relay_output_on{false};
px4::atomic<bool> relay_output_valid{false};

const char *relay_override_name(RelayOverride mode)
{
	switch (mode) {
	case RelayOverride::Auto:
		return "auto";

	case RelayOverride::ForceOff:
		return "forced off";

	case RelayOverride::ForceOn:
		return "forced on";
	}

	return "unknown";
}

bool relay_requested_on()
{
	switch (relay_override.load()) {
	case RelayOverride::ForceOn:
		return true;

	case RelayOverride::ForceOff:
		return false;

	case RelayOverride::Auto:
	default:
		return relay_auto_on.load();
	}
}

bool update_relay_output(bool force_write = false)
{
	const bool on = relay_requested_on();

	if (!force_write && relay_output_valid.load() && relay_output_on.load() == on) {
		return true;
	}

	const int ret = px4io_set_m113_relay(on);

	if (ret != PX4_OK) {
		PX4_ERR("vehicle relay IO PWM OUT 1 update failed (%i)", ret);
		return false;
	}

	relay_output_on.store(on);
	relay_output_valid.store(true);
	PX4_INFO("vehicle relay IO PWM OUT 1: GPIO %s", on ? "HIGH" : "LOW");
	return true;
}

} // namespace

bool set_relay_auto_state(bool on, bool force_write)
{
	relay_auto_on.store(on);
	return update_relay_output(force_write);
}

bool set_relay_override(RelayOverride mode)
{
	relay_override.store(mode);
	return update_relay_output(true);
}

bool relay_output_is_on()
{
	return relay_output_valid.load() && relay_output_on.load();
}

void print_relay_status()
{
	PX4_INFO("vehicle relay IO PWM OUT 1: GPIO %s, mode: %s, auto request: %s",
		 !relay_output_valid.load() ? "unknown" : relay_output_on.load() ? "HIGH" : "LOW",
		 relay_override_name(relay_override.load()),
		 relay_auto_on.load() ? "ON" : "OFF");
}
