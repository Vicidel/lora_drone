clear all;

% what we want:
% - distance and angle based on altitude with best signal and signal strength at this point
% - so we need 3D model of signal and noise
%
% todo:
% - find best angle alpha_optimal
%     - by using drone to collect data
%     - move up and down at fixed distance, get maximum signal
%     - antennas can stay straight for both
%     - can't do that yet (Ahmed still uses GW)
% - find typical values at alpha_optimal
%     - we will approach the node from this direction in the case of a 3D localization
%     - so we need to know distance as a function of signal INSIDE of this direction
%     - we can get one value from previous measures at 10m
%     - then two possibilities
%         - extrapolate based on previous results (same regression)
%         - make new measurements

% dataset, Nx3 size, N points
% in order horizontal distance, vertical distance, signal strength
dataset = [20, 10, -92;
           50, 10, -102;
           100, 10, -112;
           150, 10, -119;
           200, 10, -126];
            