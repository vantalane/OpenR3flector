# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 vantalane <mete@kestech.net>

cmake_minimum_required(VERSION 3.16)
project(openr3flector C ASM)

add_compile_definitions(STM32F411xE)

execute_process(
	COMMAND git rev-parse --short HEAD
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_HASH
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
)

if(NOT GIT_HASH)
	set(GIT_HASH "unknown")
endif()

# For others than myself, get the toolchains below
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(CPU cortex-m4)
set(MCPU "-mcpu=${CPU} -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${MCPU} -Wall -Wextra -ffunction-sections -fdata-sections -g -Og")
set(CMAKE_ASM_FLAGS "${MCPU}")
set(CMAKE_EXE_LINKER_FLAGS "${MCPU} -T${CMAKE_SOURCE_DIR}/tool_config/STM32F411VEHX_FLASH.ld -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(STM32_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/CMSIS/Include
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc
)

set(HAL_SOURCES
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
	${CMAKE_SOURCE_DIR}/../STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
)

add_executable(${PROJECT_NAME}
	src/main.c
	src/functions.c
	src/motor_control.c
	src/stm32f4xx_hal_msp.c
	src/stm32f4xx_it.c
	src/system_stm32f4xx.c
	src/uart_comms.c
	tool_config/Startup/startup_stm32f411vehx.s
	tool_config/User/sysmem.c
	tool_config/User/syscalls.c
	${HAL_SOURCES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE GIT_HASH="${GIT_HASH}")

target_include_directories(${PROJECT_NAME} PRIVATE
	${STM32_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/include
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
	COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
)

# OR try openocd
find_program(STFLASH st-flash) # stlink-tools
add_custom_target(st-flash-write
	COMMAND ${STFLASH} --reset write ${PROJECT_NAME}.bin 0x8000000
	DEPENDS ${PROJECT_NAME}
)
