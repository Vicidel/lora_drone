# LoRa localization on drones

Author: Victor Delafontaine.
Master project at Swisscom Digital Lab, supervised by the LIS-EPFL.
February to August 2019.
Supervisors: Fabrizio Schiano, Giuseppe Cocco, Alexandru Rusu.


## Drone implementation folder

This folder contains the files used to make the drone(s) fly. Also in simulation.


## Contents

This folder contains:
- drone_logs
- florian_rpi
- gmaps_api
- scripts
- server_app
- tuino_lora_beaconing
- vic_package

#### DRONE_LOGS
Contains the important logs of different flights.

#### FLORIAN_RPI
Contains the C++ code of Florian Kaufmann used on his Raspberry Pi to make the drone fly. Initially wanted to take inspiration from it, but decided to do again from scrathc as I use ROS and he didn't.

#### GMAPS_API
Contains the HTML/JS Google Maps API code used to visualize the drone flight in Maps. Also used as a GUI for giving commands.

#### SCRIPTS
Contains different scripts to ease the process of launching the simulation or compiling the ROS node...

#### SERVER_APP
Contains the Python code of the Swisscom server app. The server is connected to both the drone and the GMaps and is the one to compute the commands for the drone.

#### TUINO_LORA_BEACONING
Contains the Arduino code to put the beacon in "lost" mode when the button is pressed.

#### VIC_PACKAGE
Contains the ROS node in C++ to run on the drone.
