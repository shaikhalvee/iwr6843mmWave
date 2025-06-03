
% Housekeeping: Close figures, clear variables, and set up paths
close all;                         % Close all figure windows
clearvars;                         % Clear all variables from workspace

LOAD_CALIBRRATION  = 1;           % Flag to control if mismatch calibration is loaded

% Phase Shifter calibration file path
phaseShiftCalibFile = './input/calibrConfig/phaseShifterCalibration.mat';

% Phase Mismatch calibration file path
phaseMismatchCalibFile = './input/calibrConfig/phaseMismatchCalibration.mat';

if LOAD_CALIBRRATION
    load();  % Load calibration data if the flag is set
end
