#!/usr/bin/env python3

import sqlite3
import sys
from collections import defaultdict

# Open the rosbag database
db_path = '/home/user/cs498/src/solar_house/solar_house.db3'
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get all topics
cursor.execute("SELECT id, name, type FROM topics")
topics = cursor.fetchall()

print("=" * 80)
print("ROSBAG ANALYSIS: solar_house.db3")
print("=" * 80)
print("\nTopics in the bag:")
for topic_id, name, msg_type in topics:
    print(f"  ID: {topic_id}, Name: {name}, Type: {msg_type}")

# Analyze message timestamps for each topic
print("\n" + "=" * 80)
print("MESSAGE RATE ANALYSIS")
print("=" * 80)

for topic_id, topic_name, msg_type in topics:
    cursor.execute("""
        SELECT timestamp FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp
    """, (topic_id,))

    timestamps = [row[0] for row in cursor.fetchall()]

    if len(timestamps) < 2:
        print(f"\n{topic_name}:")
        print(f"  Message count: {len(timestamps)}")
        print(f"  Not enough messages to calculate rate")
        continue

    # Convert timestamps from nanoseconds to seconds
    timestamps_sec = [t / 1e9 for t in timestamps]

    # Calculate time differences between consecutive messages
    time_diffs = [timestamps_sec[i+1] - timestamps_sec[i]
                  for i in range(len(timestamps_sec)-1)]

    # Calculate statistics
    total_time = timestamps_sec[-1] - timestamps_sec[0]
    message_count = len(timestamps)
    avg_rate = (message_count - 1) / total_time if total_time > 0 else 0
    avg_dt = sum(time_diffs) / len(time_diffs) if time_diffs else 0
    min_dt = min(time_diffs) if time_diffs else 0
    max_dt = max(time_diffs) if time_diffs else 0

    print(f"\n{topic_name} ({msg_type}):")
    print(f"  Total messages: {message_count}")
    print(f"  Duration: {total_time:.3f} seconds")
    print(f"  Average publishing rate: {avg_rate:.2f} Hz")
    print(f"  Average dt between messages: {avg_dt*1000:.3f} ms ({avg_dt:.6f} s)")
    print(f"  Min dt: {min_dt*1000:.3f} ms")
    print(f"  Max dt: {max_dt*1000:.3f} ms")

    # Check if this is IMU or GPS related
    if 'imu' in topic_name.lower():
        print(f"  >>> IMU TOPIC IDENTIFIED <<<")
        print(f"  >>> Nominal dt would be: {1/avg_rate*1000:.3f} ms or {1/avg_rate:.6f} s <<<")
    elif 'gps' in topic_name.lower():
        print(f"  >>> GPS TOPIC IDENTIFIED <<<")
        print(f"  >>> Nominal dt would be: {1/avg_rate*1000:.3f} ms or {1/avg_rate:.6f} s <<<")

conn.close()

print("\n" + "=" * 80)
print("COMPARISON WITH CODE")
print("=" * 80)
print(f"The main template (coding_ex2.py:99) has self.dt = 0.0125")
print(f"This corresponds to: {0.0125*1000} ms or {1/0.0125} Hz")
print("=" * 80)
