cd ~/src/Firmware
export PX4_HOME_LAT=46.513372
export PX4_HOME_LON=6.563049
export PX4_HOME_ALT=400
make px4_sitl gazebo HEADLESS=1 PX4_SIM_SPEED_FACTOR=1
