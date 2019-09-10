pulseGenFMin = 485;  %MHz
pulseGenFMax = 4065; %MHz
%  our transmitter output power will be x dB
txPower = -14; %dBm
PRF = 48; %MHz.
% - our transmitting antenna gain will be x dB
txAntGainMin = 2; %dBi
txAntGainMax = 8; %dBi

% tag antennas
tagAntGainMin = 8; %dBi
tagAntGainMax = 10; %dBi
tagAmpGain = 21; %dBm

% - the attenuation through 1 foot of wet soil (max wetness we care about) will be x

% - the attenuation through another 1 foot of wet soil (heading upwards) will be x

% - the receiving antenna gain will be x
rxAntGainMin = 2; %dBi
rxAntGainMax = 8; %dBi
% Input amplifier gain is programmable in 7 steps, from -6 to 23 dB.
rxAmpGain = 23; % dBm...but check that default setting is 6, not 5!
rsSensitivity = -95 % dBm
% NOTE: HSC (high speed comparator) compares the output signal from the amplfier 
% to a DC-threshold voltage and decides whether the signal is below or above this 
% threshold in addition to providing further amplification. This additional
% amplififcation is not specified in the datasheet

% - self-inflicted interference will be x (value from 0th bin?)

% - noise at the frequencies of interest will be x [including the
% possibility of Wi-Fi or cellular signals, which at least we can measure]
% TODO: use spectrum analyzer to record typical noise from 0.4-5Ghz

% - coding gain will be x

% - therefore, overall post-decoding SINR will be x 