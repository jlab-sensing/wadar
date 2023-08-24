function [moisture] = calculateSoilMoisture(airBins, airSoilBins, soilType, d)
    % Script to calculate VWC from ToF and teros-12 sensor measurements
    c=299792458;
    resolution = 0.003790984152165; %0.004; % 
    t=((airSoilBins-airBins+d/resolution)*resolution)/c; %seconds

    % t = d/c
    
    %Values
    radar_perm = ((c*t)/d)^2;
    
    % TODO: need to redo this?
    perm_to_RAW = (0.01018)*radar_perm.^3 + (-1.479)*radar_perm.^2 + (77.47)*radar_perm + 1711; %creates pseudoRAW values from radar_perm values

    %Equations 
    %radar_calibrated = (1.059e-9)*perm_to_RAW^3 - (8.088e-6)*perm_to_RAW^2 + (2.066e-2)*perm_to_RAW - 1.727e1 %created by us
    %teros_calibrated = (1.059e-9)*RAW^3 - (8.088e-6)*RAW^2 + (2.066e-2)*RAW - 1.727e1 %calibrated by us
    if soilType == 'farm'
        radar_farmcalibrated = (5.12018081e-10)*perm_to_RAW^3 - (0.000003854251138)*perm_to_RAW^2 + (0.009950433112)*perm_to_RAW - 8.508168835941; % Using Soil Calibration for UCSC Soil
        moisture = radar_farmcalibrated;
    else if soilType == 'stanfordFarm'
        radar_farmcalibrated = (9.079e-10)*perm_to_RAW^3 - (6.626e-6)*perm_to_RAW^2 + (1.643e-2)*perm_to_RAW - 1.354e1;%created by us
        moisture = radar_farmcalibrated; 
    elseif soilType == 'stanfordSilt'
        radar_siltcalibrated = (-3.475e-10)*perm_to_RAW^3 + (2.263e-6)*perm_to_RAW^2 - (4.515e-3)*perm_to_RAW + 2.85e0;%created by us
        moisture = radar_siltcalibrated;
    elseif soilType == 'stanfordClay' 
        radar_claycalibrated = (5.916e-10)*perm_to_RAW^3 - (4.536e-6)*perm_to_RAW^2 + (1.183e-2)*perm_to_RAW - 1.017e1;%created by us
        moisture = radar_claycalibrated; 
    else
        error('need a legitimate soil type (farm, stanfordFarm, stanfordSilt, stanfordClay)'); 
    end
end