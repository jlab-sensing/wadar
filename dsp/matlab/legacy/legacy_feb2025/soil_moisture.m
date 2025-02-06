% Script to calculate VWC from ToF and teros-12 sensor measurements
c=299792458;
resolution = 0.003790984152165; %0.004; % 
airBins = 41; % number of bins from the radar to the un-burried tag
%121 151 183 240 270
airSoilBins = 240; % number of bins from the radar to the burried tag
RAW = 2314;  %RAW teros12 measurement
d=0.33; % meters sensor is buried underground
t=((airSoilBins-airBins+d/resolution)*resolution)/c; %seconds

%Values
radar_perm = ((c*t)/d)^2
teros_perm = ((2.887e-9)*RAW^3-(2.080e-5)*RAW^2+(5.276e-2)*RAW-43.39)^2

% TODO: need to redo this?
perm_to_RAW = (0.01018)*radar_perm.^3 + (-1.479)*radar_perm.^2 + (77.47)*radar_perm + 1711 %creates pseudoRAW values from radar_perm values

%Equations 
%radar_calibrated = (1.059e-9)*perm_to_RAW^3 - (8.088e-6)*perm_to_RAW^2 + (2.066e-2)*perm_to_RAW - 1.727e1 %created by us
%teros_calibrated = (1.059e-9)*RAW^3 - (8.088e-6)*RAW^2 + (2.066e-2)*RAW - 1.727e1 %calibrated by us
%radar_farmcalibrated = (9.079e-10)*perm_to_RAW^3 - (6.626e-6)*perm_to_RAW^2 + (1.643e-2)*perm_to_RAW - 1.354e1%created by us
%teros_farmcalibrated = (9.079e-10)*RAW^3         - (6.626e-6)*RAW^2         + (1.643e-2)*RAW         - 1.354e1 %calibrated by us
%radar_siltcalibrated = (-3.475e-10)*perm_to_RAW^3 + (2.263e-6)*perm_to_RAW^2 - (4.515e-3)*perm_to_RAW + 2.85e0%created by us
%teros_siltcalibrated = (-3.475e-10)*RAW^3         + (2.263e-6)*RAW^2         - (4.515e-3)*RAW         + 2.85e0 %calibrated by us
radar_claycalibrated = (5.916e-10)*perm_to_RAW^3 - (4.536e-6)*perm_to_RAW^2 + (1.183e-2)*perm_to_RAW - 1.017e1%created by us
teros_claycalibrated = (5.916e-10)*RAW^3         - (4.536e-6)*RAW^2         + (1.183e-2)*RAW         - 1.017e1 %calibrated by us

%teros_potting =   (6.771e-10)*RAW^3 - (5.105e-6)*RAW^2 + (1.302e-2)*RAW - 10.848 %from p. 8 of teros 12 manual
%radar_topp = (4.3e-6)*radar_perm^3 - (5.5e-4)*radar_perm^2 + (2.92e-2)*radar_perm - 5.3e-2
%teros_topp = (4.3e-6)*teros_perm^3 - (5.5e-4)*teros_perm^2 + (2.92e-2)*teros_perm - 5.3e-2