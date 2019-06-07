function salsaPlot(frameTot_bb, framesFFT)

frameCount = size(frameTot_bb, 2); 

% Figure 1: FFT for each bin
figure(1); im = imagesc(framesFFT);
title('Radar response across all frequencies'); 
ylabel('Range bin'); 
xlabel('Frequency');

% Figure 2: FFT of bin with largest DC response???
[~,maxRangeIndex] = max(framesFFT(:,1)); % TODO change this 1 
figure(2); plot(framesFFT(maxRangeIndex,:));
title(sprintf('Radar response of bin %i across all frequencies', maxRangeIndex)); 
ylabel('Magnitude (dB)'); 
xlabel('Frequency');

% Figure 3: ???
framesDiff = diff(frameTot_bb,[],2); 
figure(3); imagesc(db(abs(fft(framesDiff,(frameCount-1),2)))); 
title('Differential radar response across all frequencies');
ylabel('Range bin');
xlabel('Frequency'); 

% Figure 4: FFT plot for bins ranging from firstBin to lastBin 
% TODO: Add ancho vs cayenne 
firstBin = 110;
lastBin = 240; 
figure(4); plot(framesFFT(firstBin:lastBin,:)') 
title(sprintf('Radar response, bins %i-%i', firstBin, lastBin))
ylabel('Magnitude (dB)')
xlabel('Frequency');

end
    

 