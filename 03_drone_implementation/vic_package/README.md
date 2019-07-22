# LoRa localization on drones

Author: Victor Delafontaine.  
Master project at Swisscom Digital Lab, supervised by the LIS-EPFL.  
February to August 2019.  
Supervisors: Fabrizio Schiano, Giuseppe Cocco, Alexandru Rusu.  


## Folder description

This folder contains the ROS node in C++ to run on the drone.


## Contents

The source sub-folder contains:
- cJSON, to create JSON files to send
- server_app.cpp/h, to connect the drone and the server, creates the JSON and sends them using CURL
- vic_coordinates, takeoff then goes in all directions once, useful to know where is positive or negative x or y
- vic_mission_v2, main code of the project
- vic_takekoff, just takeoff, used to test code and safeties