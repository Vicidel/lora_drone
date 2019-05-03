cd catkin_ws
catkin build vic_package
source devel/setup.bash

echo " "
echo "What program do you want to run?"
echo "  0=exit"
echo "  1=vic_takeoff"
echo "  2=vic_mission"
echo "  3=vic_coordinates"
echo "  4=vic_mission_3drones"

read program

a=0
b=1
c=2
d=3
e=4

if [ $program -eq $a ]
then
	echo "Exiting now"
fi

if [ $program -eq $b ]
then
	echo "Running vic_takeoff"
	cd
	cd rosbag_files
	rosbag record -a __name:=current_bag > /dev/null 2>&1 &
	rosrun vic_package vic_takeoff
	rosnode kill /current_bag > /dev/null 2>&1
fi

if [ $program -eq $c ]
then
	echo "Running vic_mission"
	cd
	cd rosbag_files
	rosbag record -a __name:=current_bag > /dev/null 2>&1 &
	rosrun vic_package vic_mission
	rosnode kill /current_bag > /dev/null 2>&1
fi

if [ $program -eq $d ]
then
	echo "Running vic_coordinates"
	cd
	cd rosbag_files
	rosbag record -a __name:=current_bag > /dev/null 2>&1 &
	rosrun vic_package vic_coordinates
	rosnode kill /current_bag > /dev/null 2>&1
fi

if [ $program -eq $e ]
then
	echo "Running vic_mission_3drones"
	cd
	cd rosbag_files
	rosbag record -a __name:=current_bag > /dev/null 2>&1 &
	rosrun vic_package vic_mission_3drones
	rosnode kill /current_bag > /dev/null 2>&1
fi
