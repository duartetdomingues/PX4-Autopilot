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

using namespace time_literals;

bool M113::initialize_md80(uint8_t node_id, const char *name)
{
	if (!send_nmt(0x82, node_id)) {
		PX4_ERR("MD80 %s NMT pre-operational failed", name);
		return false;
	}

	sleep_servicing(100);

	// Configurar parâmetros, PDOs e salvar em memória não-volátil
	if (!configure_md80_parameters(node_id)) {
		PX4_ERR("MD80 %s parameter configuration failed", name);
		return false;
	}

	// Ler Parametros do PID Controller
	md80_read_pid_parameters(node_id);

	// Set Parametros do PID Controller
	// Default PID parameters for MD80: P=0.35, I=0.5, D=0.0, FF=1.5
	if (node_id == MD80_GEAR_ID)
	{
		// Keep the gear PID gains loaded from the MD80 non-volatile memory.
		// They can be changed and stored with `m113 gear pid ...`.
	}
	else if(node_id == MD80_ACC_ID){
		md80_set_pos_pid_parameters(node_id, 20.0f, 0.9f, 0.0f, 10.0f);
		md80_set_velocity_pid_parameters(node_id, 0.05f, 0.5f, 0.0f, 1.5f);
	}


	// Salvar parâmetros em memória não-volátil
	const uint32_t save = 0x65766173;
	PX4_INFO("Saving MD80 %s parameters to non-volatile memory", name);
	if (!sdo_write(node_id, 0x1010, 0x01, save)) {
		PX4_ERR("MD80 %s parameter store failed", name);
		return false;
	}

	sleep_servicing(3000);
	PX4_INFO("MD80 %s parameters saved to non-volatile memory", name);


	// Configurar PDOs
	if (!sdo_write(node_id, 0x6060, 0x00, MD80_MODE_SERVICE)
	    || !configure_md80_tpdos(node_id)
	    || !configure_md80_rpdos(node_id)
	) {
		PX4_ERR("MD80 %s PDO configuration failed", name);
		return false;
	}


	if (node_id == MD80_ACC_ID) {

		const int8_t set_zero1= -1;
		if (!sdo_write(node_id, 0x2003, 0x05, set_zero1)) {
			PX4_ERR("MD80 %s zero-position command failed", name);
			return false;
		}

		sleep_servicing(1000);



		int position1 = -1;
		sdo_read(node_id, 0x6064, 0x00, position1);

		PX4_INFO("MD80 %s zero-position set to %d ", name, position1);


		sleep_servicing(1000);

	}


	if(false)
	{
		switch_md80_to_operation_enabled(node_id);

		sleep_servicing(1000);

		bool service_ok = sdo_write(node_id, 0x6060, 0x00, MD80_MODE_SERVICE);
		PX4_INFO("MD80 %s mode set to service: %s", name, service_ok ? "ok" : "failed");

		sleep_servicing(1000);

		const int8_t run_calibration = 1;

		PX4_INFO("MD80 %s run-calibration command sent", name);

		if (!sdo_write(node_id, 0x2003, 0x03, run_calibration)) {
			PX4_ERR("MD80 %s run-calibration write failed", name);
			return false;
		}

		sleep_servicing(30000);
	}

	// Colocar o MD80 em modo de operação
	if (!send_nmt(0x01, node_id)) {
		PX4_ERR("MD80 %u failed to enter Operational", node_id);
		return false;
	}
	sleep_servicing(3000);

	// Colocar o MD80 em modo de idle
	if (!sdo_write(node_id, 0x6060, 0x00, MD80_MODE_IDLE)){
		PX4_ERR("MD80 %s mode set to idle failed", name);
		return false;
	}
	sleep_servicing(200);

	// Colocar o MD80 em modo de operação
	if ( !switch_md80_to_operation_enabled(node_id)){
		PX4_ERR("MD80 %s mode switch to operation enabled failed", name);
		return false;
	}
	sleep_servicing(200);

	// Colocar o MD80 em modo de posição [MD80_MODE_CYCLIC_SYNC_POSITION]
	if (!sdo_write(node_id, 0x6060, 0x00, MD80_MODE_PROFILE_POSITION)){
		PX4_ERR("MD80 %s mode set to cyclic sync position failed", name);
		return false;
	}
	sleep_servicing(1000);

	// Ler o Modo atual do MD80
	int8_t mode = -1;
	sdo_read(node_id, 0x6061, 0x00, mode);
	PX4_INFO("MD80 %s mode set to %d", name, mode);


	// set gear position to 0
	if(false)
	{
		int target_position = 0;
		if (!send_md80_target_sdo(MD80_GEAR_ID, target_position)) {
			PX4_ERR("MD80 target position SDO failed");
		}
		else {
			PX4_INFO("MD80 target position SDO sent to %d",target_position);
		}
		sleep_servicing(1000);


		// Read the gear position
		int32_t pos = -1;
		sdo_read(MD80_GEAR_ID, 0x6064, 0x00, pos);
		PX4_INFO("MD80 %s gear position: %ld", name, static_cast<long>(pos));

	}


	// TEST: set gear position to 20000
	if (false) {

		int target_position2 = 20000;
		if (!send_md80_target_sdo(MD80_GEAR_ID, target_position2)) {
			PX4_ERR("MD80 target position SDO failed");
		}
		else {
			PX4_INFO("MD80 target position SDO sent to %d",target_position2);
		}


		while (!should_exit()) {
			int pos1 = -1;
			sdo_read(MD80_GEAR_ID, 0x6064, 0x00, pos1);
				PX4_INFO("MD80 %s gear position: %ld", name, static_cast<long>(pos1));

			uint16_t status_word = 0;
			sdo_read(MD80_GEAR_ID, 0x6041, 0x00, status_word);

			PX4_INFO("MD80 %s gear status word: 0x%04x", name, status_word);

			if ((status_word & 0x0008) != 0) {
				PX4_ERR("MD80 %s FAULT active", name);
				md80_read_all_status(MD80_GEAR_ID);
			}

			if (((status_word >> 11) & 0x0001) != 0) { // 11 bit: internal limit flag
				PX4_WARN("MD80 %s internal limit flag set/latched", name);
				md80_read_all_status(MD80_GEAR_ID);
			}

			sleep_servicing(1000);
		}
	}


	if (node_id == MD80_ACC_ID) {
		_md80_target_position = 0;

	} else if (node_id == MD80_GEAR_ID) {
		_gear_target_position = 0;
	}

	PX4_INFO("MD80 %s %u initialized", name, node_id);
	return true;
}

bool M113::configure_md80_parameters(uint8_t node_id)
{

	int32_t rated_current = 0;
	int32_t rated_torque = 0;
	uint16_t max_torque = 0;
	uint16_t max_current = 0;
	uint32_t max_speed = 0;
	uint32_t pole_pairs = 0;
	float torque_constant = 0.0f;
	uint16_t torque_bandwidth = 0;
	uint8_t shutdown_temperature = 0;
	float gear_ratio = 0.0f;

	if (node_id == MD80_ACC_ID) {
		rated_current = 3800;
		rated_torque = 3000;
		max_torque = 3000;
		max_current = 3421;
		max_speed = 233;
		pole_pairs = 14;
		torque_constant = 0.120f;
		torque_bandwidth = 500;
		shutdown_temperature = 80;
		gear_ratio = 0.166666f;

	} else if (node_id == MD80_GEAR_ID) {
		rated_current = 7200; // 7.2 A
		rated_torque = 8300; // 8.3 Nm
		max_torque = 3000; // 3x rated torque for short duration
		max_current = 3222; // 3.22x rated current for short duration
		max_speed = 148; // 148 rpm
		pole_pairs = 21;
		torque_constant = 0.123f; // 0.123 Nm/A
		torque_bandwidth = 500; // 500 Hz
		shutdown_temperature = 80; // 80 C
		gear_ratio = 0.1f; // 10:1 reduction

	} else {
		PX4_ERR("MD80 %u unknown node ID for parameter configuration", node_id);
		return false;
	}

	const int32_t min_software_position_limit = INT32_MIN;
	const int32_t max_software_position_limit = INT32_MAX;
	bool ok = true;

	ok &= configure_md80_parameter(node_id, 0x6075, 0x00, rated_current);
	ok &= configure_md80_parameter(node_id, 0x6076, 0x00, rated_torque);
	ok &= configure_md80_parameter(node_id, 0x6072, 0x00, max_torque);
	ok &= configure_md80_parameter(node_id, 0x6073, 0x00, max_current);
	ok &= configure_md80_parameter(node_id, 0x6080, 0x00, max_speed);
	ok &= configure_md80_parameter(node_id, 0x2000, 0x01, pole_pairs);
	ok &= configure_md80_parameter(node_id, 0x2000, 0x02, torque_constant);
	ok &= configure_md80_parameter(node_id, 0x2000, 0x05, torque_bandwidth);
	ok &= configure_md80_parameter(node_id, 0x2000, 0x07, shutdown_temperature);
	ok &= configure_md80_parameter(node_id, 0x2000, 0x08, gear_ratio);
	ok &= configure_md80_parameter(node_id, 0x607D, 0x01, min_software_position_limit);
	ok &= configure_md80_parameter(node_id, 0x607D, 0x02, max_software_position_limit);

	return ok;
}



bool M113::configure_md80_tpdos(uint8_t node_id)
{
	const uint8_t transmission_type = 0xFE; // asynchronous / event-driven
	const uint16_t inhibit_time = 0;        // units: 100 us, 0 = disabled
	const uint16_t event_timer = 100;       // ms

	// 1) Meter em Pre-Operational para configurar PDOs por SDO
	if (!send_nmt(0x80, node_id)) {
		PX4_ERR("MD80 %u failed to enter pre-operational", node_id);
		return false;
	}

	sleep_servicing(100);

	for (uint8_t pdo = 0; pdo < 4; ++pdo) {
		const uint16_t comm_index = 0x1800 + pdo;

		// TPDO1: 0x180 + node_id
		// TPDO2: 0x280 + node_id
		// TPDO3: 0x380 + node_id
		// TPDO4: 0x480 + node_id
		const uint32_t cob_id = 0x180 + (static_cast<uint32_t>(pdo) * 0x100) + node_id;
		const uint32_t disabled_cob_id = cob_id | 0x80000000UL;

		// 2) Desativar TPDO antes de alterar parâmetros
		if (!sdo_write(node_id, comm_index, 0x01, disabled_cob_id)) {
			PX4_ERR("MD80 %u TPDO%u disable failed", node_id, pdo + 1);
			return false;
		}

		// 3) Configurar parâmetros
		if (!sdo_write(node_id, comm_index, 0x02, transmission_type) ||
		    !sdo_write(node_id, comm_index, 0x03, inhibit_time) ||
		    !sdo_write(node_id, comm_index, 0x05, event_timer)) {
			PX4_ERR("MD80 %u TPDO%u parameter config failed", node_id, pdo + 1);
			return false;
		}

		// 4) Reativar TPDO
		if (!sdo_write(node_id, comm_index, 0x01, cob_id)) {
			PX4_ERR("MD80 %u TPDO%u enable failed", node_id, pdo + 1);
			return false;
		}

		PX4_INFO("MD80 %u TPDO%u enabled: COB-ID=0x%03lx, event=%u ms",
			 node_id,
			 pdo + 1,
			 static_cast<unsigned long>(cob_id),
			 event_timer);

		sleep_servicing(50);
	}

	// 5) Meter em Operational para começar a transmitir PDOs
	if (!send_nmt(0x01, node_id)) {
		PX4_ERR("MD80 %u failed to enter operational", node_id);
		return false;
	}

	sleep_servicing(100);

	return true;
}


bool M113::configure_md80_rpdos(uint8_t node_id)
{
	const uint8_t transmission_type = 0xFF;

	const uint32_t rpdo4_disabled = 0x80000500UL + node_id;
	const uint32_t rpdo3_disabled = 0x80000400UL + node_id;
	const uint32_t rpdo3_enabled  = 0x00000400UL + node_id;
	const uint32_t rpdo2_disabled = 0x80000300UL + node_id;
	const uint32_t rpdo1_disabled = 0x80000200UL + node_id;

	// Meter o MD80 em Pre-Operational para configurar PDOs
	if (!send_nmt(0x80, node_id)) {
		PX4_ERR("MD80 %u failed to enter Pre-Operational", node_id);
		return false;
	}

	sleep_servicing(100);

	// Configurar RPDOs
	if (!(sdo_write(node_id, 0x1403, 0x01, rpdo4_disabled)
	      && sdo_write(node_id, 0x1402, 0x01, rpdo3_disabled)
	      && sdo_write(node_id, 0x1402, 0x02, transmission_type)
	      && sdo_write(node_id, 0x1402, 0x01, rpdo3_enabled)
	      && sdo_write(node_id, 0x1401, 0x01, rpdo2_disabled)
	      && sdo_write(node_id, 0x1400, 0x01, rpdo1_disabled))) {
		PX4_ERR("MD80 %u RPDO configuration failed", node_id);
		return false;
	}

	sleep_servicing(100);

	// Meter o MD80 em Operational para aceitar/enviar PDOs
	if (!send_nmt(0x01, node_id)) {
		PX4_ERR("MD80 %u failed to enter Operational", node_id);
		return false;
	}

	sleep_servicing(100);

	PX4_INFO("MD80 %u RPDO3 enabled and node is Operational", node_id);

	return true;
}
