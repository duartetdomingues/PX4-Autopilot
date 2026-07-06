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

#include <cstdint>

enum class RelayOverride : uint8_t {
	Auto,
	ForceOff,
	ForceOn,
};

bool set_relay_auto_state(bool on, bool force_write = false);
bool set_relay_override(RelayOverride mode);
bool relay_output_is_on();
void print_relay_status();
