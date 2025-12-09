#!/bin/bash
# Helper script to view ar_drive debug log in real-time
# The log file is created when ENABLE_PRINT = true in ar_drive_control.cpp

LOG_FILE="/tmp/ar_drive_debug.log"

if [ ! -f "$LOG_FILE" ]; then
    echo "Log file not found: $LOG_FILE"
    echo "Make sure to run: ros2 launch ar_control ar_control.launch.py ecat:=true"
    echo "with ENABLE_PRINT = true in ar_drive_control.cpp"
    exit 1
fi

echo "Watching ar_drive debug log..."
echo "Log file: $LOG_FILE"
echo "Press Ctrl+C to stop"
echo "----------------------------------------"

tail -f "$LOG_FILE"
