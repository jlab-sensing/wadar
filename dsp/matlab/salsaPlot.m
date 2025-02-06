function salsaPlot(frameTot_bb, framesFFT, runCount, startRange, endRange)

numFrames = size(frameTot_bb, 2); 

% Figure 1: FFT for each bin
str = strcat('Run #', num2str(runCount));
sgtitle(str);
%TODO: Add title for four figures

subplot(2,2,1);
im = imagesc(framesFFT);
title('Radar response across all frequencies'); 
ylabel('Range bin'); 
xlabel('Frequency');

% Figure 2: FFT of bin with largest DC response???
subplot(2,2,2);
[~,maxRangeIndex] = max(framesFFT(:,1)); % TODO change this 1 
plot(framesFFT(maxRangeIndex,:));
title(sprintf('Radar response of bin %i across all frequencies', maxRangeIndex)); 
ylabel('Magnitude (dB)'); 
xlabel('Frequency');

% Figure 3: ???
subplot(2,2,3);
framesDiff = diff(frameTot_bb,[],2); 
imagesc(db(abs(fft(framesDiff,(numFrames-1),2)))); 
title('Differential radar response across all frequencies');
ylabel('Range bin');
xlabel('Frequency'); 

% Figure 4: FFT plot for bins ranging from firstBin to lastBin 
% TODO: Add ancho vs cayenne 
subplot(2,2,4);
plot(framesFFT(startRange:endRange,:)') 
title(sprintf('Radar response, bins %i-%i', startRange, endRange))
ylabel('Magnitude (dB)')
xlabel('Frequency');

end
    

 