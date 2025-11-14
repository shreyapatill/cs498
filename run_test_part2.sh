#!/bin/bash
set -e

cd /home/shreyap7/cs498/ros2_ws

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Remove old results
rm -f results_part2.txt

# Run part2 node in background
ros2 run mobile_robotics part2 > part2.log 2>&1 &
NODE_PID=$!

# Give node time to start
sleep 3

# Play rosbag for 35 seconds
timeout 35 ros2 bag play src/solar_house/solar_house.db3 > bag2.log 2>&1 || true

# Give node time to finish writing
sleep 2

# Stop the node
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

# Check results
if [ -f results_part2.txt ]; then
    LINE_COUNT=$(wc -l < results_part2.txt)
    echo "Part 2 Results: $LINE_COUNT lines"
    echo "First 3 lines:"
    head -3 results_part2.txt
    echo "Last 3 lines:"
    tail -3 results_part2.txt
elif [ -f src/results_part2.txt ]; then
    LINE_COUNT=$(wc -l < src/results_part2.txt)
    echo "Part 2 Results: $LINE_COUNT lines (in ros2_ws/src/)"
    echo "First 3 lines:"
    head -3 src/results_part2.txt
    echo "Last 3 lines:"
    tail -3 src/results_part2.txt
else
    echo "ERROR: results_part2.txt not found!"
    echo "Checking logs..."
    tail -20 part2.log
    exit 1
fi
