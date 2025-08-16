# Makefile for RCMowerControllerAIV1.ino (Arduino Leonardo)

BOARD_TAG = arduino:avr:leonardo
PORT = /dev/cu.usbmodem1101
SKETCH = RCMowerControllerAIV1.ino
BUILD_DIR = build

all: compile

compile:
	arduino-cli compile --fqbn $(BOARD_TAG) --output-dir $(BUILD_DIR) .

verify: compile

upload: compile
	arduino-cli upload -p $(PORT) --fqbn $(BOARD_TAG) --input-dir $(BUILD_DIR) .

monitor:
	arduino-cli monitor -p $(PORT) -c baudrate=115200

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all compile verify upload clean monitor 