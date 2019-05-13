#!/bin/bash

# watchdog process
mainpid=$$
(sleep 1; kill $mainpid) &
watchdogpid=$!

sshpass -p alex1234 ssh -p 10000 pi@localhost "cd /home/pi/latency_test; ./run $1 $2 $3"

kill $watchdogpid