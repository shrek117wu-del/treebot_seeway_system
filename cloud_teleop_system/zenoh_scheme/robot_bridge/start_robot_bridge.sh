#!/bin/bash
# Simple helper script to start the zenoh-bridge-dds on the Jetson Orin NX
# Make sure you have installed zenoh-bridge-dds via apt or cargo:
# sudo apt install zenoh-bridge-dds

echo "Starting Zenoh ROS 2 Bridge (Robot Mode)..."
echo "Make sure to edit robot.json5 with your actual cloud server IP."

zenoh-bridge-dds -c robot.json5
