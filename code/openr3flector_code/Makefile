# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 vantalane <mete@kestech.net>

.PHONY: all configure build clean rebuild flash

BUILD_DIR := build
JOBS := $(shell nproc)
PROJECT_NAME := openr3flector

configure:
	mkdir -p $(BUILD_DIR)
	cmake -S . -B $(BUILD_DIR)

build: configure
	cmake --build $(BUILD_DIR) -- -j$(JOBS)

all: build

clean:
	cmake --build $(BUILD_DIR) --target clean || true
	rm -rf $(BUILD_DIR)/*

rebuild: clean configure build

flash: build
	# uses your target .bin created by post build
	st-flash --reset write $(BUILD_DIR)/$(PROJECT_NAME).bin 0x8000000
