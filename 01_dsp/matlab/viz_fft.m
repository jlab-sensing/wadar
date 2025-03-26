function viz_fft(captureFT, tagFT, frameRate)
% viz_fft(captureFT, tagFT, frameRate)
%
% Function to visualize the FFT of Novelda radar data captures. This function reads in
% the pre-processed FFT of the radar data frames and the FFT of the tag signal and
% visualizes the data. 
%
% Inputs:
%   captureFT: FFT of the radar frames
%   tagFT: FFT of the tag signal
%
% Outputs:
%   None

frameCount = width(captureFT);

figure
captureFT(:, 1:2) = ones(512, 2);       % First 2 frames of capture are much higher than the rest 
                                        % because much of the captured "image" is stationary.
x = (1:512)';
y = (1:frameCount) / frameCount * frameRate;
[xMat, yMat] = meshgrid(x, y);
zMat = abs(captureFT(:, 1:frameCount))';
surf(xMat, yMat, zMat, 'EdgeColor', 'none');
ax = gca;       % Get current axes
xlim([0 512])

xlabel('Range Bins')
ylabel('Frequency')
zlabel('Magnitude')
title('3D FFT')

figure
plot(tagFT)
xlabel('Range Bins')
ylabel('Magnitude')
title('Isolated tag frequency')

end