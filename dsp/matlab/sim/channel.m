%%%%%%%%% define excitation signal (baseband)
% design FIR filter to filter noise to half of Nyquist rate
bandwidth = 3e9;
Fs = 39e9; %sampling frequency of radar 
b = bandwidth/Fs; %bandwidth in terms of normalized frequency units of cycles/sample
f = fir1(64, b/2, 'low');
% generate Gaussian (normally-distributed) white noise
n = randn(1e4, 1);
% apply to filter to yield bandlimited noise
nb = filter(f,1,n);
% scale variance after filtering
var = 1.0;  % just an example  
scale = sqrt(var)/std(nb);
nb = scale*nb;  % nb has variance 'var'
sXY = dlmread('VNA/spectrum.csv',';',1,0);
f = sXY(:,1);
raw = db2mag(sXY(:,2));
zeroForced = 1e-12*ones(length(raw),1);
for i=1:length(f)
    if raw(i) > mean(raw)
        zeroForced(i) = raw(i);
    end
end
zeroForced = [1e-12*ones(f(1)/(f(2)-f(1)),1); zeroForced(2:end)];
zeroForced = [zeroForced; 1e-12*ones((3e9/(f(2)-f(1)))-length(zeroForced),1)];
zeroForced = [flip(zeroForced(1:78:end)); zeroForced(1:78:end)];
%TODO: subsample
%figure(); plot(db(zeroForced)); title(' '); grid on

%%%%%%%%% modulate to Fc
Fc = 1.5e9;
% TODO

%%%%%%%%% define the channel (in the time domain)
% freq[Hz];re:Trc1_S14;im:Trc1_S14; 
sXY = dlmread('VNA/tagHigh.csv',';',1,0);
freq = sXY(:,1);
sYX = dlmread('VNA/taglow.csv',';',1,0);
cpxHi = zeros(length(freq),1);
cpxLo = zeros(length(freq),1);

for i = 1:length(freq) 
    cpxHi(i) = complex(sXY(i,2),sXY(i,3));
    cpxLo(i) = complex(sYX(i,2),sYX(i,3));
end
% TODO: extend this to cover 0-500Mhz
cpxHi = [ones(65,1)*cpxHi(1); cpxHi];
cpxLo = [ones(65,1)*cpxLo(1); cpxLo];
chanHi = [flip(cpxHi); cpxHi];
chanLo = [flip(cpxLo); cpxLo];
figure(); plot(db(abs(chanHi))); hold on;
plot(db(abs(chanLo)));
legend("tag hi","tag lo")
title('H'); grid on

% alternate between tag on and tag off at 50-80Hz

%TODO: how do we efficiently align the size of the frequency domain for
%multiplication? 

%%%%%%%%% send signal through the channel (convolution)
%TODO: is this dB math right? I don't think so...
figure; plot(db(abs(chanHi).*zeroForced)); hold on;
plot(db(abs(chanLo).*zeroForced))
%plot(db(zeroForced))
grid on
legend('tag hi','tag lo')
%%%%%%%%% downconvert(?)
%%%%%%%%% downconvert(?)

%%%%%%%%% post-process to look for peak (convolve, FFT, e c)