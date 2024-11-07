#!/usr/bin/env bash

# Remove the existing build directory
echo "Removing the existing build directory..."
rm -rf build install

# Build the project
echo "Building the project..."
colcon build

# Install the project
echo "Installing the project..."
source install/setup.bash

sed -i 's/\#!\/usr\/bin\/python3/\#!\/usr\/bin\/env python3/g' $(pwd)/install/line_detection_pipeline/lib/line_detection_pipeline/*