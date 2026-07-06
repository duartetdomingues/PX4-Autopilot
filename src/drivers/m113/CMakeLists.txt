############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
############################################################################

# M113 uses the STM32H7 FDCAN backend directly for classic CANopen frames.
# Keep it independent from the UAVCAN daemon because both own the CAN peripheral.
# FMUv6X builds both FDCAN interfaces and reserves TIM2 for this backend.
set(M113_UAVCAN_NUM_IFACES 2)
set(M113_UAVCAN_TIMER 2)
set(LIBDRONECAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan)
set(LIBDRONECAN_DIR_DRIVERS ${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers)
set(M113_UAVCAN_DRIVER_DIR ${LIBDRONECAN_DIR_DRIVERS}/stm32h7/driver)
set(M113_DSDLC_DIR ${LIBDRONECAN_DIR}/dsdl)
set(M113_DSDLC_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/dsdlc_generated)

if(DEFINED config_uavcan_timer_override)
	set(M113_UAVCAN_TIMER ${config_uavcan_timer_override})
endif()

if(DEFINED config_uavcan_num_ifaces)
	set(M113_UAVCAN_NUM_IFACES ${config_uavcan_num_ifaces})
endif()

px4_add_git_submodule(TARGET git_m113_dsdl PATH ${M113_DSDLC_DIR})
px4_add_git_submodule(TARGET git_m113_pydronecan PATH ${LIBDRONECAN_DIR}/libuavcan/dsdl_compiler/pydronecan)

file(GLOB_RECURSE M113_DSDLC_INPUT_FILES ${M113_DSDLC_DIR}/uavcan/*.uavcan)
add_custom_command(
	OUTPUT m113_uavcan_dsdlc_run.stamp
	COMMAND
		${PYTHON_EXECUTABLE} ${LIBDRONECAN_DIR}/libuavcan/dsdl_compiler/libuavcan_dsdlc
			--outdir ${M113_DSDLC_OUTPUT} ${M113_DSDLC_DIR}/uavcan
	COMMAND ${CMAKE_COMMAND} -E touch m113_uavcan_dsdlc_run.stamp
	DEPENDS ${M113_DSDLC_INPUT_FILES}
	COMMENT "M113 UAVCAN dsdl compiler"
	)
add_custom_target(m113_uavcan_dsdlc DEPENDS m113_uavcan_dsdlc_run.stamp)
add_dependencies(m113_uavcan_dsdlc git_m113_dsdl git_m113_pydronecan)

px4_add_module(
	MODULE drivers__m113
	MAIN m113
	STACK_MAIN 6144
	SRCS
		m113.cpp
		m113_actuators.cpp
		m113_canopen.cpp
		m113_gear.cpp
		m113_md80.cpp
		m113_md80_utils.cpp
		m113_relay.cpp
		m113_thomson.cpp
		${LIBDRONECAN_DIR}/libuavcan/src/uc_data_type.cpp
		${LIBDRONECAN_DIR}/libuavcan/src/uc_error.cpp
		${LIBDRONECAN_DIR}/libuavcan/src/node/uc_global_data_type_registry.cpp
		${M113_UAVCAN_DRIVER_DIR}/src/uc_stm32h7_can.cpp
		${M113_UAVCAN_DRIVER_DIR}/src/uc_stm32h7_clock.cpp
		${M113_UAVCAN_DRIVER_DIR}/src/uc_stm32h7_thread.cpp
	COMPILE_FLAGS
		-Wno-cast-align
		-Wno-deprecated-copy
		-Wno-address-of-packed-member
	INCLUDES
		${LIBDRONECAN_DIR}/libuavcan/include
		${M113_DSDLC_OUTPUT}
		${M113_UAVCAN_DRIVER_DIR}/include
	DEPENDS
		m113_uavcan_dsdlc
	MODULE_CONFIG
		module.yaml
	)

target_compile_definitions(drivers__m113 PRIVATE
	UAVCAN_CPP_VERSION=UAVCAN_CPP03
	UAVCAN_DRIVER=uavcan_stm32h7
	UAVCAN_IMPLEMENT_PLACEMENT_NEW=1
	UAVCAN_MEM_POOL_BLOCK_SIZE=48
	UAVCAN_NO_ASSERTIONS
	UAVCAN_NUM_IFACES=${M113_UAVCAN_NUM_IFACES}
	UAVCAN_PLATFORM=generic
	UAVCAN_STM32H7_NUTTX=1
	UAVCAN_STM32H7_NUM_IFACES=${M113_UAVCAN_NUM_IFACES}
	UAVCAN_STM32H7_TIMER_NUMBER=${M113_UAVCAN_TIMER}
	)
