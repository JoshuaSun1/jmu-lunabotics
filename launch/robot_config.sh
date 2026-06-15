#!/usr/bin/env bash

# Master robot launch configuration.
# Use true/false for feature toggles. Environment variables can override these
# defaults when launching scripts from the command line.

ROBOT_CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_CAMERA_INFO_FILE="${ROBOT_CONFIG_DIR}/../sensor/webcam/webcam_calibration.yaml"

# Sensors
: "${ENABLE_CAMERA:=true}"
: "${VIDEO_DEVICE:=/dev/video2}"
: "${CAMERA_NAME:=webcam}"
: "${CAMERA_INFO_FILE:=${DEFAULT_CAMERA_INFO_FILE}}"
: "${CAMERA_INFO_URL:=}"
: "${ENABLE_APRILTAG:=true}"
: "${APRILTAG_TAG_ID:=0}"
: "${APRILTAG_TAG_FAMILY:=36h11}"
: "${APRILTAG_TAG_SIZE_METERS:=0.250}"

# Visualization
: "${ENABLE_VIEW_IMAGE_RAW:=true}"
