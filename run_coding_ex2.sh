#!/bin/bash

cd /home/shreyap7/cs498/ros2_ws

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Path to rosbag
ROSBAG_PATH="/home/shreyap7/cs498/2025_coding_exercise2/2022_coding_exercise2/solar_house_v3"

# Cleanup function
cleanup() {
    echo ""
    echo "Interrupted! Stopping EKF node and cleaning up..."
    kill -INT $NODE_PID 2>/dev/null || true
    kill $BAG_PID 2>/dev/null || true
    wait $NODE_PID 2>/dev/null || true
    wait $BAG_PID 2>/dev/null || true
    exit 0
}

# Set trap for cleanup on interrupt (Ctrl+C)
trap cleanup INT TERM

echo "Starting EKF node (cod_ex2)..."
echo "This will display: xy trajectory, euler angles, gyroscope bias, accelerometer bias, and covariance of position"
echo ""

# Create plots directory
PLOTS_DIR="cp2_plots"
mkdir -p "$PLOTS_DIR"
echo "Plots will be saved to: $PLOTS_DIR/"
echo ""

# Run cod_ex2 node in background
ros2 run mobile_robotics cod_ex2 > coding_ex2.log 2>&1 &
NODE_PID=$!

# Give node time to start
sleep 3

echo "Playing rosbag: $ROSBAG_PATH"
# Play rosbag
ros2 bag play "$ROSBAG_PATH" > bag.log 2>&1 &
BAG_PID=$!

# Wait for bag to finish
wait $BAG_PID 2>/dev/null || true

echo ""
echo "Rosbag finished. Waiting a few seconds for final processing..."
sleep 3

echo "Saving plots and stopping EKF node..."
# Send SIGINT to node to trigger cleanup and plot saving
kill -INT $NODE_PID 2>/dev/null || true

# Wait for node to finish saving
wait $NODE_PID 2>/dev/null || true

echo ""
echo "Done! Plots saved to $PLOTS_DIR/"
ls -lh "$PLOTS_DIR"/*.png 2>/dev/null || echo "No plots found"
echo ""

