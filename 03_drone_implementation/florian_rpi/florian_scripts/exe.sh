#!/bin/bash
# evaluate latency in function of excitation frequency

for lag in {13..20..1} # this is actualy latency = 5*lag in ms -> max 100ms
do
	for omega in {1..3..2} # omega = 2*pi*freq
	do
		for window in {0..2..2} #this is actualy window = 5*window + 5ms -> range of {5ms - 105ms}
		do
			cd /home/flo/src/Firmware/build/posix_sitl_default/logs/2018-07-30/ # change data before to run
			rm ./*
			#run gazebo
			px4.sh
			# connect to rpi
			rpi.sh $omega $lag $window
			# kill rpi as we are done
			kill_rpi.sh

			windows=$((5*$window + 5)) #this is actualy window = 5*window + 5ms -> range of {5ms - 105ms}
			latency=$((5 * $lag)) # latency = 5*lag in ms
			frequency=$(bc <<< "scale=2; $omega/6.28") # freq = omega / 2pi
			OUTPUT="$(ls -1)"
			NEW=("lat""${latency[@]}""ms_""freq""${frequency[@]}""hz_window""${windows[@]}""ms.ulg")
			mv $OUTPUT ~/flight\ logs/Bash\ logs/${NEW[@]}
		done
	done
done

