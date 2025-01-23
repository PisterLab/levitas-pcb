# by default, compile (if need be) then upload and start program
all: build-then-reload

# compile firmware for Pico board but do not run
build:
	mkdir -p build
	rm -rf build/*
	cd build && cmake ..
	cd build && make levitas

# upload and verify only
# requires board to start in BOOTSEL mode
load:
	picotool load -v build/levitas.elf

# upload, verify, then start program
# works either if board is in BOOTSEL mode
# or if it is running with default USB communication (pico_stdio_usb library included)
reload:
	picotool load -v -x -f build/levitas.elf
build-then-reload: build
	picotool load -v -x -f build/levitas.elf

# remove build directory and files
clean:
	rm -rf build

# make these Makefile rules run every time
# (instead of make's only-run-if-new thing)
.PHONY: all build load reload build-then-reload clean
