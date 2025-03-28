function PlotDualTag(captureFT, tag1FT, tag2FT, tag1Name, tag2Name, frameRate)
% PlotDualTag(captureFT, tag1FT, tag2FT, tag1Name, tag2Name, frameRate)
%
% Function to visualize the FFT of the radar data captures and the isolated
% tag frequencies.
%
% Inputs:
%   captureFT: FFT of the radar frames
%   tag1FT: FFT of the first tag frequency
%   tag2FT: FFT of the second tag frequency
%   tag1Name: Name of the first tag frequency
%   tag2Name: Name of the second tag frequency
%   frameRate: Frame rate in Hz
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
xlim([0 512])

xlabel('Range Bins')
ylabel('Frequency')
zlabel('Magnitude')
title('3D FFT')

figure
subplot(2,1,1)
plot(tag1FT); hold on
plot(tag2FT)
legend([string(tag1Name), string(tag2Name)])
xlabel('Range Bins')
ylabel('Magnitude')
title('Raw isolated tag frequencies')

subplot(2,1,2)
plot(tag1FT ./ max(tag1FT)); hold on
plot(tag2FT ./ max(tag2FT))
legend([string(tag1Name), string(tag2Name)])
xlabel('Range Bins')
ylabel('Normalized magnitude')
title('Normalized isolated tag frequencies')

end