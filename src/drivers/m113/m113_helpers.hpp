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

#include <cstddef>
#include <cstdint>

namespace m113
{

template<typename T>
void put_le(uint8_t *destination, T value)
{
	for (size_t i = 0; i < sizeof(T); ++i) {
		destination[i] = static_cast<uint8_t>(value >> (8 * i));
	}
}

inline uint32_t get_le32(const uint8_t *source)
{
	return static_cast<uint32_t>(source[0])
	       | (static_cast<uint32_t>(source[1]) << 8)
	       | (static_cast<uint32_t>(source[2]) << 16)
	       | (static_cast<uint32_t>(source[3]) << 24);
}

inline int scale_joystick(uint16_t raw)
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

} // namespace m113
