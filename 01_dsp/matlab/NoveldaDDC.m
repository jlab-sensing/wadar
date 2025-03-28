function basebandSignal = NoveldaDDC(rfSignal, chipSet, PGen, Fs, CF)
% basebandSignal = NoveldaDDC(rfSignal, chipSet, PGen, Fs, Fc)
%
% Function to apply a digital downcovert (DDC) to a high frequency radar
% signal. Brings signal to baseband frequencies and provides an analytic
% signal (i.e. I & Q, in-phase & quadrature, outputs)
%
% Inputs:
%   rfSignal: high-frequency sampled UWB radar signal
%    chipSet: 'X1-IPG0' or 'X1-IPG1' for NVA6100 or 'X2' for NVA620x
%       PGen: 0 = slow, 1 = nominal, 3 = fast, for 'X1'
%             0-10 (CF = 5.3GHz to 8.8 GHz), for 'X2'
%         Fs: sampling rate in Hz (extracted directly from chip)
%         Fc: center frequency for custom down-conversion (optional input)
%
% Outputs:
%   basebandSignal: downconverted, i.e. baseband, and filtered IQ radar 
%                   signal, note: use abs() on output to get envelope
%                   magnitude
%
% Created 5/11/2015
% Copyright FlatEarth, Inc.
%
% 5/12/2015 Update to include optional Center Frequency (CF) input.
%
% 3/1/2016 Update to base frameSize on length of rfsignal instead of
% NoveldaChipParams function
%
% 12/4/2017 Update hard-coded Fs for X4 chip 
%

% Don't need to specify Fs if using X4 chip...
if nargin < 4 && strcmpi(chipSet, 'x4')
    % X4 attributes (always the same)
    Fs = 23.328e9;
    Sampler = '6mm';
else
    % Calculate Sampler for X1/X2
    if Fs < 10e9
        Sampler = '4cm';
    elseif Fs < 30e9
        Sampler = '8mm';
    else
        Sampler = '4mm';
    end
end

% If custom, user-specified CF (center frequency) is not provided...
if nargin < 5
    % Call NoveldaChipParams function to get chip parameters
    CF = NoveldaChipParams(chipSet, PGen, Sampler);
end

% Find size of rf signal
[row, col] = size(rfSignal);

% Frame Size according to input rf signal
if row > col
    frameSize = row;
else
    frameSize = col;
end

% Digital Down-Convert parameters (normalized frequency index)
freqIndex = CF  / Fs * frameSize;
freqIndex=freqIndex;
% Generate the complex sinusoid LO (local oscillator) to mix into the signal 
t = linspace(0, 1, frameSize);
%phi=-pi/3;
phi=0;
LO = sin(2 * pi * freqIndex * t+phi)  +  1i * cos(2 * pi * freqIndex * t+phi);
% Adjust LO size, just in case
LO = LO.';

% Digital Downconvert (the DDC) via direct multiplication
% subtracting the mean removes DC offset
basebandSignal = (rfSignal-mean(mean(rfSignal))) .* LO;

% % LPF Design to eliminate the upper mixing frequencies after the DDC (21 
% % tap LPF, may need to be tuned based on the rate of desired roll-off)
% fn = bw_Hz / 2 / Fs;            % normalized cutoff frequency
% M = 20;                         % filter order, equal to # of filter taps - 1
% lpfWeights = zeros(1, M + 1);   % allocate memory
% for n = 0:M
%     if n ~= M/2
%         % LPF based on sinc function (21 taps)
%         lpfWeights(n + 1) = sin((2* pi * fn) * (n - M/2)) / (pi * (n -  M/2));
%     else
%         % Needed for filter with odd number of taps
%         lpfWeights(n + 1) = 2 * fn;
%     end
% end
% window = filterDesign(M + 1, 'hamming');    % create window to smooth filter
% filterWeights = window .* lpfWeights;       % direct multiply weights for final weights

% LPF Design to eliminate the upper mixing frequencies after the DDC (21- tap hamming window)
M = 20;                                             % filter order, equal to # of filter taps - 1
% M = 10;
% window = filterDesign(M + 1, 'hamming');            % window
window = hamming(M + 1); 
filterWeights = window / sum(window(1:(M/2 + 1)));  % normalized weights

% Baseband signal using convolution (provides downcoverted, filtered analytic signal)
basebandSignal = conv(basebandSignal, filterWeights, 'same');


