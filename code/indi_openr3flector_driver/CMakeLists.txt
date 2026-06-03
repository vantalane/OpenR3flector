# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 vantalane <mete@kestech.net>

cmake_minimum_required(VERSION 3.13)
project(r3_indi_driver C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../indi/cmake_modules/")

find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)

set(INDI_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}../indi/libs/indibase
)

# Add SGP4 lib
set(SGP4_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../sgp4/src/cpp")

set(SGP4_SOURCES
	"${SGP4_SRC_DIR}/SGP4.c"
	"${SGP4_SRC_DIR}/TLE.cpp"
)

set_source_files_properties(
	"${SGP4_SRC_DIR}/SGP4.c"
	PROPERTIES LANGUAGE C
)

set(SGP4_INCLUDE_DIR "${SGP4_SRC_DIR}")
add_library(sgp4_static STATIC ${SGP4_SOURCES})
target_include_directories(sgp4_static PUBLIC "${SGP4_INCLUDE_DIR}")

# if SGP4.c needs specific compile flags, add them here:
# target_compile_options(sgp4_static PRIVATE ...)
include_directories(${SGP4_SRC_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})
include_directories(${NOVA_INCLUDE_DIR})
include_directories(${EV_INCLUDE_DIR})

include(CMakeCommon)

add_executable(indi_r3_telescope
	${CMAKE_CURRENT_SOURCE_DIR}/mount_driver.cpp
)

target_link_libraries(indi_r3_telescope
	indidriver
	${INDI_LIBRARIES}
	${NOVA_LIBRARIES}
	${GSL_LIBRARIES}
	sgp4_static
)

# install(TARGETS indi_r3_telescope RUNTIME DESTINATION bin)
