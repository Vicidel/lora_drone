#!/bin/bash
# log to ssh and kill everything, px4 included

sleep 40
sshpass -p alex1234 ssh -p 10000 pi@localhost "pkill run"
pkill px4