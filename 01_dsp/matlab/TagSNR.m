function SNRdb = TagSNR(captureFT, freqIndex, peakBin)
% SNRdb = TagSNR(captureFT, freqIndex, peakBin)
%
% Function to calculate the signal-to-noise ratio (SNR) in decibels of a
% tag signal in the FFT of radar data captures.
%
% Inputs:
%   captureFT: FFT of the radar frames
%   freqIndex: Index of the tag frequency in the FFT
%   peakBin: Peak bin of the tag signal
%
% Outputs:
%   SNRdb: Signal-to-noise ratio in decibels

signalMag = abs(captureFT(peakBin, freqIndex));

noiseFreq = round(freqIndex*0.945 : freqIndex*0.955);

noiseMag = mean(abs(captureFT(peakBin, noiseFreq))); 

SNR = signalMag / noiseMag; 
SNRdb = 10 * log10(SNR); 

end


