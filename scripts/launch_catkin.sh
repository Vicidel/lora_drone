cd catkin_ws
catkin build vic_package
source devel/setup.bash

echo " "
echo "What program do you want to run?"
echo "  0=exit"
echo "  1=vic_takeoff"
echo "  2=vic_mission"
echo "  3=vic_coordinates"

read program

a=0
b=1
c=2
d=3

if [ $program -eq $a ]
then
	echo "Exiting now"
fi

if [ $program -eq $b ]
then
	echo "Running vic_takeoff"
	rosrun vic_package vic_takeoff
fi

if [ $program -eq $c ]
then
	echo "Running vic_mission"
	rosrun vic_package vic_mission
fi

if [ $program -eq $d ]
then
	echo "Running vic_coordinates"
	rosrun vic_package vic_coordinates
fi
