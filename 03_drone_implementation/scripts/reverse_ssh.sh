#!/bin/bash

if [ "$#" != "3" ]; then
  echo "Usage error, order is 1:address 2:port 3:password"
  exit
fi

REMOTE_ADDR=$1
REMOTE_PORT=$2
PASSWORD=$3

while true; do
  sshpass -p "$PASSWORD" \
  ssh -o ServerAliveInterval=60 \
  -o ServerAliveCountMax=2 \
  -o StrictHostKeyChecking=no \
  -o UserKnownHostsFile=/dev/null \
  -o ConnectTimeout=15 \
  -o LogLevel=ERROR \
  -N -R 2222:localhost:22 $REMOTE_ADDR -p $REMOTE_PORT
  sleep 60
done
 
