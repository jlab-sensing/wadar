function [freqTag, freqTagHar] = calculateTagFrequencies(tagHz, frameRate, numTrials)

freqTag = tagHz / frameRate * numTrials; 
freqTagHar = (frameRate - tagHz) / frameRate * numTrials; 

if tagHz == 80
    if numTrials <= frameRate * 30 % <= 30s
        freqTag = freqTag + 1; 
        freqTagHar = freqTagHar + 1;    
    elseif numTrials == frameRate * 100 % 100s
        freqTag = freqTag + 0; 
        freqTagHar = freqTagHar + 2;  
    elseif numTrials == frameRate * 300 % 300s
        freqTag = freqTag -1; 
        freqTagHar = freqTagHar + 3; 
    else
        error('unrecognized capture duration') 
    end

elseif tagHz == 50
     if numTrials <= frameRate * 30 % <= 30s
        freqTag = freqTag + 1; 
        freqTagHar = freqTagHar + 1;   
    elseif numTrials == frameRate * 100 % 100s
        freqTag = freqTag + 1; 
        freqTagHar = freqTagHar + 1;   
    elseif numTrials == frameRate * 300 % 300s
        freqTag = freqTag + 0; 
        freqTagHar = freqTagHar + 2; 
    else
        error('unrecognized capture duration') 
    end

end

end