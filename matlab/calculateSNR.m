% function to calculate signal to noise ratio.
% calculates ratio of peak bin amplitude at desired 
% frequency vs an average of a few irrelevant frequencies 

% Args
% ft: ft (bins x frequencies) - magnitude or complex
% freq: desired frequency 
% bin: selected bin 

function SNRdb = calculateSNR(ft, freq, bin)

signalMag = abs(ft(bin, freq));

noiseFreq = round(freq*0.945 : freq*0.955);

noiseMag = mean(abs(ft(bin, noiseFreq))); 

SNR = signalMag / noiseMag; 
SNRdb = 10 * log10(SNR); 

end


