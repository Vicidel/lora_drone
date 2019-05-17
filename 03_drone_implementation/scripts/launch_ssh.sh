DRONE_PORT=$1
until sshpass -p alex1234 ssh localhost -p $DRONE_PORT; do
	echo "Trying to SSH into RPi"
	sleep 5
done
