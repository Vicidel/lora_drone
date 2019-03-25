#!/bin/bash
# run px4 gazebo simulation

# watchdog process
mainpid=$$
(sleep 1; kill $mainpid) &
watchdogpid=$!

cd /home/flo/src/Firmware/
make posix gazebo

kill $watchdogpid