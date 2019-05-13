#!/bin/bash

cd /home/flo/drone/TEST\ MOTORS/logs

for lag in {0..20..1} # this is actualy latency = 5*lag in ms -> max 100ms
do
	for window in {2..20..2} #this is actualy window = 5*window + 5ms -> range of {5ms - 105ms}
	do
		windows=$((5*$window + 5)) #this is actualy window = 5*window + 5ms -> range of {5ms - 105ms}
		latency=$((5 * $lag)) # latency = 5*lag in ms

		frequency=$(python f.py)
		echo frequency

		name=("lat""${latency[@]}""ms_""freq""${frequency[@]}""hz_window""${windows[@]}""ms.csv")

		python analyse.py $name $latency $frequency $windows
	done
done