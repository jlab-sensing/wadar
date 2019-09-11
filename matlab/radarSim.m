s11File = 's41saturatedFarm30cm.csv';
pullseShape = '';

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

sXY = dlmread(s11File,';',1,0);
f = sXY(:,1);

cpx = zeros(500,1);
for i = 1:500 
    cpx(i) = complex(sXY(i,2),sXY(i,3));
end
tagAntS11 = db(abs(cpx));
cpx = zeros(500,1);

figure(); plot(f,tagAntS11); title('Tag S11'); ylim([-100,10]); grid on

% - the attenuation through 1 foot of wet soil (max wetness we care about)
% - assume attenuation heading upwards will be symmetric
vnaTXPow = 0; % dBm
for i = 1:500 
    cpx(i) = complex(sXY(i,4),sXY(i,5));
end
magS41 = db(abs(cpx));

figure(); plot(f,magS41); title('path loss'); ylim([-100,10]); grid on

% simplified model: assume 100% of the received power is re-radiated, but
% in real life look here? https://people.ece.uw.edu/nikitin_pavel/papers/APmag_2006.pdf


% - the receiving antenna gain will be x
rxAntGainMin = 2; %dBi
rxAntGainMax = 8; %dBi
% Input amplifier gain is programmable in 7 steps, from -6 to 23 dB.
rxAmpGain = 23; % dBm...but check that default setting is 6, not 5!
rsSensitivity = -95; % dBm
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