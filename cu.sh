#!/bin/bash
[ "$1" = "c" ] && { arduino-cli compile --fqbn arduino:avr:mega; } || { arduino-cli compile --fqbn arduino:avr:mega && sudo arduino-cli upload -p /dev/ttyUSB0 -b arduino:avr:mega; }
