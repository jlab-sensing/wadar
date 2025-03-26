function [tagFT, tagIndex] = tag_index(captureFT, frameRate, tagHz)
% [tagFT, tagIndex] = tag_index(captureFT, frameRate, tagHz)
%
% Function to find the tag frequency index in the FFT of the radar data
% captures. 
%
% Inputs:
%   captureFT: FFT of the radar frames
%   frameRate: Frame rate in Hz
%   tagHz: Tag frequency in Hz
%
% Outputs:
%   tagFT: FFT of the tag signal

frameCount = width(captureFT);

% Find Tag FT
freqTag = tagHz / frameRate * frameCount;
tagFT = abs(captureFT(:, freqTag));
tagIndex = freqTag;
for i = (freqTag-2:1:freqTag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tagFT)
            tagFT = temp;
            tagIndex = i;
    end
end
tagFT = smoothdata(tagFT, 'movmean', 10);

end