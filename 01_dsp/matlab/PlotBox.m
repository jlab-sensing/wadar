function PlotBox(rangeBins, dataName)
% PlotBox(x, dataName)
%
% Visualize the range bin consistency of the radar data.
%
% Inputs:
%   rangeBins: Range bins of the radar data
%   dataName: Name of the radar data
%
% Outputs:
%   None

figure
boxplot(rangeBins)
ylabel('Range bins')
xlabel(dataName)
title('Box plot of range bin consistency')

end