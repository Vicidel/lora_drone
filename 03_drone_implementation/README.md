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
- gmaps_api
- matlab_drone_logs_analysis
- misc
- scripts
- server_app
- solidworks_parts
- tuino_lora_beaconing
- vic_package

#### DRONE_LOGS
Contains the important logs of different flights.

#### GMAPS_API
Contains the HTML/JS Google Maps API code used to visualize the drone flight in Maps. Also used as a GUI for giving commands.

#### MATLAB_DRONE_LOGS_ANALYSIS
Contains the files used to decode the ROSBAG files present in the *drone\_logs* folder.

#### MISC
Contains miscellaneous files.

#### SCRIPTS
Contains different scripts to ease the process of launching the simulation or compiling the ROS node...

#### SERVER_APP
Contains the Python code of the Swisscom server app. The server is connected to both the drone and the GMaps and is the one to compute the commands for the drone.

#### SOLIDWORKS_PARTS
Contains the differents parts 3D-printed and laser-cut at LIS for this project.

#### TUINO_LORA_BEACONING (and ...V2)
Contains the Arduino code to put the beacon in "lost" mode when the button is pressed. The v2 is for Micha's beacon with onboard GPS to have a ground truth.

#### VIC_PACKAGE
Contains the ROS node in C++ to run on the drone.
