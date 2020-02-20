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

%%%%%%%%% modulate to Fc
Fc = 1.5e9;
% TODO

%%%%%%%%% define the channel (in the time domain)

% alternate between tag on and tag off at 50-80Hz

%%%%%%%%% send signal through the channel (convolution)
%

%%%%%%%%% downconvert(?)

%%%%%%%%% post-process to look for peak (convolve, FFT, e c)