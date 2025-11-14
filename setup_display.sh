#!/bin/bash
# Setup DISPLAY for WSL2 with WSLg
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
echo "DISPLAY set to: $DISPLAY"
echo "You can now run rviz2 or other GUI applications"
