#!/usr/bin/env bash
set -eo pipefail

echo "========================================"
echo "Starting crawler ROS launch"
date
echo "User: $(id)"
echo "Hostname IPs: $(hostname -I || true)"
echo "========================================"

source /opt/ros/jazzy/setup.bash
source /home/ros/crawler-venv/bin/activate
source /home/ros/crawler-pi/crawler/install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unknown}"
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "VIRTUAL_ENV=${VIRTUAL_ENV:-none}"

echo "Waiting for /dev/i2c-1..."
for i in {1..20}; do
  if [ -e /dev/i2c-1 ]; then
    echo "Found /dev/i2c-1"
    break
  fi

  echo "Waiting for /dev/i2c-1... attempt ${i}/20"
  sleep 1
done

if [ ! -e /dev/i2c-1 ]; then
  echo "ERROR: /dev/i2c-1 not found"
  exit 1
fi

echo "Waiting for /dev/ttyUSB0..."
for i in {1..30}; do
  if [ -e /dev/ttyUSB0 ]; then
    echo "Found /dev/ttyUSB0"
    break
  fi

  echo "Waiting for /dev/ttyUSB0... attempt ${i}/30"
  sleep 1
done

if [ ! -e /dev/ttyUSB0 ]; then
  echo "ERROR: /dev/ttyUSB0 not found, not starting crawler launch"
  exit 1
fi

echo "Device permissions:"
ls -l /dev/ttyUSB0 || true
ls -l /dev/i2c-1 || true

echo "Starting launch file..."
exec ros2 launch crawler_bringup crawler.launch.py
