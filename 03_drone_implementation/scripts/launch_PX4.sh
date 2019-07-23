cd ~/src/Firmware
export PX4_HOME_LAT=46.521333
export PX4_HOME_LON=6.536219
export PX4_HOME_ALT=400
make px4_sitl gazebo HEADLESS=1 PX4_SIM_SPEED_FACTOR=1
