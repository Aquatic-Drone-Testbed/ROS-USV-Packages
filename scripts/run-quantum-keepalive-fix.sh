#!/bin/bash

# Kill all child processes on exit
#trap "echo 'Caught SIGINT or SIGTERM. Killing all child processes...'; kill 0; exit" SIGINT SIGTERM
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

echo "Starting keepalive script (runs forever)..."
/home/ws/scripts/quantum-fix-scripts/run-quantum-keepalive.sh &
KEEPALIVE_PID=$!
echo "Keepalive PID: $KEEPALIVE_PID"

sleep 20

# Start the first instance of the rotating script
echo "Starting worker script..."
/home/ws/scripts/quantum-fix-scripts/run-quantum-keepalive-chain.sh &
WORKER_PID=$!
echo "Worker PID: $WORKER_PID"

# Loop to restart the worker script every 180 seconds
while true; do
    sleep 30

    echo "Restarting worker script..."

    echo "Stopping current worker script (PID $WORKER_PID)..."
    kill -9 "$WORKER_PID"

    sleep 2  # Small delay before restart

    echo "Starting new worker script..."
    /home/ws/scripts/quantum-fix-scripts/run-quantum-keepalive-chain.sh &
    WORKER_PID=$!
    echo "New worker PID: $WORKER_PID"
done