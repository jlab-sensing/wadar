% normalizeX4 - Normalizes Novelda UWB receive signal (X4) to a DAC value
% and normalizes the radar signal to an absolute voltage value. This is
% based on the DAC voltage thresholder, 0-1.2 volts.
%
% [normalVoltFrame, normalDACFrame, meanVoltFrame] = normalizeX4(counterValueSig, settings, removeDC)
%
% Parameters:
%    counterValueSig: raw measurement signal in counter values
%          x4settings: settings, e.g. iterations, pps, dacMin etc., of raw 
%                     measurement signal
%           removeDC: boolean flag to remove DC bias/mean from the
%                     normalized voltage signal
%   
% Returns (in order):
%    normalVoltFrame: radar signal frame normailzed to an absolute voltage
%                     value. If "removeDC" flag is true, then the mean of the signal is
%                     removed before it is returned.
%    normalDACFrame: radar signal frame normalized to a DAC value between
%                    0 and 2047.
%
%
% Created 8-10-2017
%
function [normalVoltFrame, normalDACFrame] = normalizeX4(counterValSig, x4Settings, removeDC)

% Don't remove DC bias if not specified (Default value)
if ~exist('removeDC', 'var')
    removeDC = false;
end

% Check if radarSettings is a containers.Map class
if isobject(x4Settings)
    % Rename and clear variable
    mapSettings = x4Settings; 
    x4Settings = [];
    
    % Iterations
    if isKey(mapSettings, 'Iterations')
        x4Settings.Iterations = mapSettings('Iterations');   
    end
    
    % PulsesPerStep 
    if isKey(mapSettings, 'PPS')
        x4Settings.PPS = mapSettings('PPS');
    end
    
    if isKey(mapSettings, 'DACStep')
        x4Settings.DACStep = mapSettings('DACStep');         % DACStep
    end
    
    % DACMin
    if isKey(mapSettings, 'DACMin')
        x4Settings.DACMin = mapSettings('DACMin');
    end
    
    % DACMax
    if isKey(mapSettings, 'DACMax')
        x4Settings.DACMax = mapSettings('DACMax');
    end
end

% Constants
DacUpperLimit = 2047;
rxVolts = 1.2;
    
% Normalize to a DAC Value between 0 and 2047
noffset = (DacUpperLimit - x4Settings.DACMax + x4Settings.DACMin) / 2;
nfactor = x4Settings.Iterations * x4Settings.PPS / x4Settings.DACStep;
normalDACFrame = counterValSig / nfactor + noffset;

% Normalize/scale to a Voltage Value between 0 and 1.04 Volts
normalVoltFrame = normalDACFrame / DacUpperLimit * rxVolts;

% Remove DC bias/mean if flag is set
if removeDC
    normalVoltFrame = detrend(normalVoltFrame, 'constant');
end