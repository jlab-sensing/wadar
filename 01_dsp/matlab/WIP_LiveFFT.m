function [outFFT] = WIP_LiveFFT(frameTotFlat)

    frameTot = reshape(frameTotFlat, 512, 2000);
    
    captureFT = fft(frameTot, 2000, 2); 
    
    outFFT = max(abs(captureFT));
    outFFT(1:100) = zeros(1, 100);
    outFFT(end-99:end) = zeros(1, 100);
    
    PLOT_FLAG = true;
    if PLOT_FLAG == true
        figure(1)
        plot(outFFT); 
    end

end