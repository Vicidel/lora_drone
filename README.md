# LoRa localization on drones

Author: Victor Delafontaine.
Master project at Swisscom Digital Lab, February to August 2019.


## Project description

The goal of the project is to localize an beacon on the ground emitting LoRa messages. The message are received by a gateway mounted on one or several drones.

## Contents

The folder available are as follows:
-  json_backup
- matlab_json_analysis
- matlab_results
- matlab_signal_noise
- matlab_sim
- matlab_torus_carac
- server_app
- tuino_lora_calibration
- tuino_torus_carac

#### JSON_BACKUP
Contains the JSON files received for different data collections. Please refer to the contained *what\_is\_what.txt* for more informations on file contents.

#### MATLAB_JSON_ANALYSIS
Contains the analysis of the first big collections done from the Swisscom lab. The function *decode\_json.m* can be used to extract the data from a JSON file.

#### MATLAB_RESULTS
Contains the results of the signal characterisation. In the *coeff* files are stored the *a* and *b*  coefficients of the exponential decay between distance and signal strength as well as between attenuation and angle. The different functions to use them are also present in this folder.

#### MATLAB_SIGNAL_NOISE
Contains the 3D signal and noise characterization.  Also the equation system to decorrelate the Swisscom data from the collection height using the angle attenuation exponential curve.

#### MATLAB_SIM
Contains the scripts for running the drone simulations. Could be with one or three drones.

#### MATLAB_TORUS_CARAC
Contains the scripts for asserting the antenna parameters and matching them with the datasheet. Also the fit done to obtain the exponential coefficients.

#### SERVER_APP
Contains the Python app running on the Swisscom server to store the incoming messages into a Mongo database.

#### TUINO_LORA_CALIBRATION and TUINO_TORUS_CARAC
Contains the Arduino script to run on the Gimasi ONE to send messages using a finite state machine linked to the distances or to the angles.
