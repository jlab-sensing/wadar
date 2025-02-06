s11File = 'siltyLoamS41.csv';
radarPulse = 'chipotleFreqMag.mat';
fps = 200;
tagFreq = 50; % hz
captureDuration = 10; % seconds

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
% TODO: does the NVA620x note apply to the NVA100 also?
% NVA620x use coherent integration to achieve processing gain and
% the level of processing gain increase with higher integration. 
% Increased integration can be achieved by increasing the number of 
% pulses per step, by programming the PulsesPerStep configuration 
% register, or by increasing the number of iterations, by programming
% the Iterations configuration register. The total integration is the
% product of these two values. 
% The number of pulses to read for an individual DAC step value.  
% PulsesPerStep number of pulses are sampled and integrated
pulsesPerStep = 8; 
iterations = 16; % The total number of sweeps to execute between the DAC extremes
% TODO: do DAC min, max and step size contribute to the gain?
% Higher DACStep values means less averaging and lower sensitivity but higher speed and vice versa.
% - our transmitting antenna gain will be x dB
txAntGainMin = 2; %dBi
txAntGainMax = 8; %dBi


% tag antennas
tagAntGainMin = 8; %dBi
tagAntGainMax = 10; %dBi
tagAmpGain = 10; %dBm %TODO: make this a plot based off the datasheet(s)

sXY = dlmread(s11File,';',1,0);
f = sXY(:,1);

%cpx = zeros(500,1);
%for i = 1:500 
%    cpx(i) = complex(sXY(i,2),sXY(i,3));
%end
%tagAntS11 = db(abs(cpx));
cpx = zeros(500,1);

% TODO: y-axis units?
%figure(); plot(f,tagAntS11); title('Tag S11'); ylim([-100,10]); grid on

% - the attenuation through 1 foot of wet soil (max wetness we care about)
% - assume attenuation heading upwards will be symmetric
vnaTXPow = 0; % dBm
for i = 1:500 
    cpx(i) = complex(sXY(i,2),sXY(i,3));
end
magS41 = db(abs(cpx)); % TODO: should this be PSD instead?

% TODO: y-axis units?
figure(); plot(f,magS41); title('S41 (above to below ground'); ylim([-100,10]); grid on

% simplified model: assume 100% of the received power is re-radiated, but
% in real life look here? https://people.ece.uw.edu/nikitin_pavel/papers/APmag_2006.pdf

% - the receiving antenna gain will be x
rxAntGainMin = 2; %dBi
rxAntGainMax = 8; %dBi
% Input amplifier gain is programmable in 7 steps, from -6 to 23 dB.
rxAmpGain = 23; % dBm...but check that default setting is 6, not 5!

% TODO: subtract VNA S21 from the periodogram
% - reduce periodogram frequency range to 300Mhz-8ghz
truncRadarFreq = freq(4:103);
truncRadarPSD = 10*log10(psdx(4:103));
% - undersample VNA S21 datapoints so it has same length as truncated
% periodogram
truncVNAFreq = f(1:5:end);
% TODO: fix this PSD conversion
truncVNAPSD = 10*log10(2*(db2mag(magS41(1:5:end)).^2));
% - subtract VNA S21*2 from periodogram, apply rxAmpGain and processingGain
% The amount of processing gain is approximately doubled
% with twice the integration, resulting in an SNR enhancement of 3 dB.
thresholderGain = 31.7609; % dB https://pdfs.semanticscholar.org/d712/7aa60173f1c80b247cb01b7bc25d091c7e6f.pdf
processGain = 10*log10(iterations*pulsesPerStep);
roundTripOn = (truncRadarPSD + 2*truncVNAPSD) + rxAmpGain + tagAmpGain + thresholderGain + processGain; 
roundTripOff = (truncRadarPSD + 2*truncVNAPSD) + rxAmpGain + thresholderGain - tagAntGainMax; %TODO: is this right?

% Plot single frame
% TODO: add RX threshold and RX freq range filtering?
figure(); plot(truncRadarFreq,roundTripOn); grid on; title("received radar signal of tag bin, tag on"); ylim([-100 0]); xlim([pulseGenFMin*1e6 pulseGenFMax*1e6])
hold on; hline = refline([0 -95]);
set(hline,'color','r');
set(hline,'LineStyle','--');
figure(); plot(truncRadarFreq,roundTripOff); grid on; title("received radar signal of tag bin, tag OFF"); ylim([-100 0]); xlim([pulseGenFMin*1e6 pulseGenFMax*1e6])
hline = refline([0 -95]);
set(hline,'color','r');
set(hline,'LineStyle','--');

% - figure out the deal with sensitivity 
% sensitivity in what sense? Integrated across the whole spectrum, or
% instantaneously for any bin?
% Also, Receiver sensitivity at 10 dB SNR. Radar Settings were: 
% DACMin = 4014, DACMax = 4178, PulsesPerStep = 100, Iterations = 10.
rxSensitivity = -95; % dBm
% - add "tag toggling" 
% does everything need to be put back into time domain?
% how to represent the non-tag bins (put in frames from a capture?)
% TODO: take a capture with radar over the soil bin (do two with antenna both
% grounded and open circuit?)

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