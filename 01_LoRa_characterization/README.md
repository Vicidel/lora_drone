# LoRa localization on drones

Author: Victor Delafontaine.
Master project at Swisscom Digital Lab, supervised by the LIS-EPFL.
February to August 2019.
Supervisors: Fabrizio Schiano, Giuseppe Cocco, Alexandru Rusu.


## LoRa characterization folder

This folder contains the files used to characterize the LoRa signal.


## Contents

This folder contains:
- json_backup
- matlab_json_analysis
- matlab_results
- matlab_signal_noise
- matlab_torus_carac
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

#### MATLAB_TORUS_CARAC
Contains the scripts for asserting the antenna parameters and matching them with the datasheet. Also the fit done to obtain the exponential coefficients.

#### TUINO_LORA_CALIBRATION and TUINO_TORUS_CARAC
Contains the Arduino script to run on the Gimasi ONE to send messages using a finite state machine linked to the distances or to the angles.
