function [peakStrength, peakLocation] = WIP_LiveFFT(newFrameBB)
    persistent framesBB n
    if isempty(n)
        n = 1;
        framesBB = zeros(512, 2000);
    end
    
    if (n > 2000)
        n = 1;
    end
    framesBB(:, n) = newFrameBB;

    peakStrength = -1;
    peakLocation = -1;
    
    if mod(n, 500) == 0
        captureFT = fft(framesBB, 2000, 2); 
        
        liveOutput = max(abs(captureFT));
        liveOutput(1:100) = zeros(1, 100);
        liveOutput(end-99:end) = zeros(1, 100);
        
        PLOT_FLAG = true;
        if PLOT_FLAG == true
            figure(1)
            plot(liveOutput); hold on;
        end

        [peakStrength, peakLocation] = max(liveOutput);
        peakLocation = peakLocation / 10;
    end

    n = n + 1;
end