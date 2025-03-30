function [peakStrength, peakLocation] = WIP_LiveFFT(newFrameBB)
    persistent framesBB n
    if isempty(n)
        n = 1;
        framesBB = zeros(512, 2000);
    end
    
    if (n > 2000)
        n = 1;
    end
    framesBB(:, n:n+1000-1) = newFrameBB;
    
    captureFT = fft(framesBB, 2000, 2); 
    
    liveOutput = max(abs(captureFT));
    liveOutput(1:100) = zeros(1, 100);
    liveOutput(end-99:end) = zeros(1, 100);
    
    PLOT_FLAG = true;
    if PLOT_FLAG == true
        figure(1)
        plot(liveOutput); 
    end

    [peakStrength, peakLocation] = max(liveOutput);
    peakLocation = peakLocation / 10;

    n = n + 500;
end