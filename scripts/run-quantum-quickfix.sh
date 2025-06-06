#!/bin/bash

# Kill all child processes on exit (on Ctrl+C or script termination)
#trap "echo 'Caught SIGINT or SIGTERM. Killing all child processes...'; kill 0; exit" SIGINT SIGTERM

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

echo "Launching first script..."
/home/ws/scripts/run-quantum.sh &
PID1=$!

sleep 60

echo "Switching to daisy chain script..."
echo "Stopping radar..."
ros2 topic pub /radar_control std_msgs/String '{data: stop_scan}' -1

/home/ws/scripts/quantum-fix-scripts/run-quantum-chain.sh &
PID2=$!

# Let both run for 5 seconds (overlap)
sleep 5

echo "Killing first script..."
kill "$PID1"
echo "Restarting radar..."
ros2 topic pub /radar_control std_msgs/String '{data: start_scan}' -1

# Initialize variables
CURRENT_PID=$PID2
CURRENT_NAME="run-quantum-chain.sh"

# Loop with 180s cycles and 5s overlap
while true; do
    sleep 180  # Let current script run alone for 180s

    echo "Switching to next daisy chain instance..."
    echo "Stopping radar..."
    ros2 topic pub /radar_control std_msgs/String '{data: stop_scan}' -1

    /home/ws/scripts/quantum-fix-scripts/run-quantum-chain.sh &
    NEW_PID=$!
    NEW_NAME="run-quantum-chain.sh"

    # Overlap for 5s
    sleep 5

    echo "Killing previous daisy chain script (PID $CURRENT_PID)..."
    kill "$CURRENT_PID"
    echo "Restarting radar..."
    ros2 topic pub /radar_control std_msgs/String '{data: start_scan}' -1

    # Update tracking vars
    CURRENT_PID=$NEW_PID
    CURRENT_NAME=$NEW_NAME
done
