s11File = 's41saturatedFarm30cm.csv';
radarPulse = 'chipotleFreqMag.mat';

load(radarPulse);
td = td/1000-4.4; % scale ADC sampled value to match datasheet

figure(); plot(td(1:135)); title('radar pulse time domain'); grid on;
Fs = 40e9; % in hz
N = length(td);
xdft = fft(td);
xdft = xdft(1:N/2+1);
psdx = (1/((Fs/1e6)*N)) * abs(xdft).^2; % in dBm/MHz
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(td):Fs/2;
figure();plot(freq,10*log10(psdx))
grid on
title('Periodogram Using FFT')
xlabel('Frequency (Hz)')
ylabel('Power/Frequency (dBm/MHz)')


pulseGenFMin = 485;  %MHz
pulseGenFMax = 4065; %MHz
%  our transmitter output power will be x dB
txPower = -14; %dBm?
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

% TODO: y-axis units?
figure(); plot(f,tagAntS11); title('Tag S11'); ylim([-100,10]); grid on

% - the attenuation through 1 foot of wet soil (max wetness we care about)
% - assume attenuation heading upwards will be symmetric
vnaTXPow = 0; % dBm
for i = 1:500 
    cpx(i) = complex(sXY(i,4),sXY(i,5));
end
magS41 = db(abs(cpx)); % TODO: should this be PSD instead?

% TODO: y-axis units?
figure(); plot(f,magS41); title('S21 (above to below ground'); ylim([-100,10]); grid on

% simplified model: assume 100% of the received power is re-radiated, but
% in real life look here? https://people.ece.uw.edu/nikitin_pavel/papers/APmag_2006.pdf

% TODO: subtract VNA S21 from the periodogram
% - reduce periodogram frequency range to 300Mhz-8ghz
truncRadarFreq = freq(4:103);
truncRadarMag = 10*log10(psdx(4:103));
% - undersample VNA S21 datapoints so it has same length as truncated
% periodogram
truncVNAFreq = f(1:5:end);
truncVNAMag = magS41(1:5:end);
% - subtract VNA S21*2 from periodogram, apply rxAmpGain
roundTrip = (truncRadarMag + 2*truncVNAMag) + 23; %TODO: is this right?
figure(); plot(truncRadarFreq,roundTrip); grid on; title("received radar signal?")
% - figure out the deal with sensitivity 
% - figure out the deal with PRF and integration 
% - add "tag toggling" 

% - the receiving antenna gain will be x
rxAntGainMin = 2; %dBi
rxAntGainMax = 8; %dBi
% Input amplifier gain is programmable in 7 steps, from -6 to 23 dB.
rxAmpGain = 23; % dBm...but check that default setting is 6, not 5!
% sensitivity in what sense? Integrated across the whole spectrum, or
% instantaneously for any bin?
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