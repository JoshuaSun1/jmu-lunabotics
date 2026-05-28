#!/usr/bin/env bash

# Master robot launch configuration.
# Use true/false for feature toggles. Environment variables can override these
# defaults when launching scripts from the command line.

# Sensors
: "${ENABLE_CAMERA:=true}"
: "${VIDEO_DEVICE:=/dev/video2}"

# Visualization
: "${ENABLE_VIEW_IMAGE_RAW:=true}"
