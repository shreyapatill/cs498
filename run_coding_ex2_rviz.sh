#!/bin/bash

cd /home/shreyap7/cs498/ros2_ws

# Setup DISPLAY for WSL2 with WSLg
export DISPLAY=:0
# Force software rendering for OpenGL (required for WSL2)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Path to rosbag
ROSBAG_PATH="/home/shreyap7/cs498/2025_coding_exercise2/2022_coding_exercise2/solar_house_v3"

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping all processes..."
    kill $NODE_PID 2>/dev/null || true
    kill $BAG_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    wait $NODE_PID 2>/dev/null || true
    wait $BAG_PID 2>/dev/null || true
    wait $RVIZ_PID 2>/dev/null || true
    exit 0
}

# Set trap for cleanup on interrupt (Ctrl+C)
trap cleanup INT TERM

echo "Starting EKF node (cod_ex2)..."
echo "This will publish odometry to /odom and path to /odom/path"
echo ""

# Run cod_ex2 node in background
ros2 run mobile_robotics cod_ex2 > coding_ex2.log 2>&1 &
NODE_PID=$!

# Give node time to start and publish initial transform
echo "Waiting for node to initialize and publish transform..."
sleep 5

# Verify that odom frame exists
echo "Checking if odom frame exists..."
timeout 3 ros2 run tf2_ros tf2_echo odom base_link > /dev/null 2>&1 &
TF_CHECK_PID=$!
sleep 2
kill $TF_CHECK_PID 2>/dev/null || true

echo "Starting rviz2..."
echo "DISPLAY=$DISPLAY"
echo "Using software rendering (LIBGL_ALWAYS_SOFTWARE=1)"
# Run rviz2 with config file (ensure DISPLAY and software rendering are set)
DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330 rviz2 -d odometry_visualization.rviz > rviz.log 2>&1 &
RVIZ_PID=$!

# Give rviz time to start and connect
sleep 3

echo "Playing rosbag: $ROSBAG_PATH"
# Play rosbag
ros2 bag play "$ROSBAG_PATH" > bag.log 2>&1 &
BAG_PID=$!

echo ""
echo "Rosbag is playing. Trajectory should be visible in rviz2."
echo "Press Ctrl+C to stop."
echo ""

# Wait for user interrupt or bag to finish
wait $BAG_PID 2>/dev/null || true

echo ""
echo "Rosbag finished. RViz is still running - take your screenshot now!"
echo "Press Ctrl+C when done to exit."
echo ""

# Keep everything running until user presses Ctrl+C
# The cleanup function will handle stopping everything
while kill -0 $NODE_PID 2>/dev/null; do
    sleep 1
done


