function VWC = SenseTagVWC(wetPeakBin, airPeakBin, soilType, distance)
% VWC = proc_vwc(wetPeakBin, airPeakBin, soilType, distance)
%
% Function calculates VWC from ToF sensor measurements.
%
% Inputs:
%       wetPeakBin: Peak bin of backscatter tag covered by wet soil.
%       airPeakBin: Peak bin of backscatter tag uncovered by soil.
%       soilType: Type of soil.
%       distance: Distance between backscatter tag and surface in meters.
%
% Outputs:
%       VWC: Volumetric water content (in decimal form).

c=299792458;
resolution = 0.003790984152165; % 0.004 m
t= ((wetPeakBin - airPeakBin + distance/resolution) * resolution)/c; %seconds

%Values
radar_perm = ((c*t)/distance).^2;

% TODO: need to redo this?
perm_to_RAW = (0.01018)*radar_perm.^3 + (-1.479)*radar_perm.^2 + (77.47)*radar_perm + 1711; %creates pseudoRAW values from radar_perm values

if soilType == 'farm'
    radar_farmcalibrated = (5.12018081e-10)*perm_to_RAW.^3 - (0.000003854251138)*perm_to_RAW.^2 + (0.009950433112)*perm_to_RAW - 8.508168835941; % Using Soil Calibration for UCSC Soil
    VWC = radar_farmcalibrated;
elseif soilType == 'stanfordFarm'
    radar_farmcalibrated = (9.079e-10)*perm_to_RAW.^3 - (6.626e-6)*perm_to_RAW.^2 + (1.643e-2)*perm_to_RAW - 1.354e1;%created by us
    VWC = radar_farmcalibrated; 
elseif soilType == 'stanfordSilt'
    radar_siltcalibrated = (-3.475e-10)*perm_to_RAW.^3 + (2.263e-6)*perm_to_RAW.^2 - (4.515e-3)*perm_to_RAW + 2.85e0;%created by us
    VWC = radar_siltcalibrated;
elseif soilType == 'stanfordClay' 
    radar_claycalibrated = (5.916e-10)*perm_to_RAW.^3 - (4.536e-6)*perm_to_RAW.^2 + (1.183e-2)*perm_to_RAW - 1.017e1;%created by us
    VWC = radar_claycalibrated; 
else
    error('ERROR: Need a legitimate soil type (farm, stanfordFarm, stanfordSilt, stanfordClay)'); 
end
