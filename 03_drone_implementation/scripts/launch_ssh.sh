until sshpass -p alex1234 ssh localhost -p 2222; do
	echo "Trying to SSH into RPi"
	sleep 5
done
