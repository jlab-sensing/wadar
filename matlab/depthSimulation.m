tx_power = 6.3; %dBm
leakage = -45;%-85; %dB for UWB in 
% determined by bits the radar RX ADC has, 
% db = 20xlog10(2^{bits})
dynamic_range = 20*log10(2^11); %66dB for the X4 
%number of meters before signal strength drops 10db
penetration_depth_dry  = [0.4  0.25  0.19  0.15  0.12  0.1   0.09  0.07  0.06  0.05];  %meters, for 7Ghz
penetration_depth_wet  = [0.19 0.07  0.03  0.025 0.018 0.013 0.009 0.007 0.006 0.005]; %meters, for 7Ghz
% using gabriel & gabriel penetration depth for muscle tissue...agar is?
penetration_depth_agar = [0.04 0.026 0.018 0.012 0.009 0.007 0.006 0.005 0.004 0.003]; %meters, for 7Ghz

% the signal needs to be louder than this for detection
threshold = leakage - dynamic_range;

units = ((threshold-tx_power)/(-10))/2;
% attenuation of signal
max_dry = [];
for p = penetration_depth_dry
    max_dry = [max_dry; units*p];
end

max_wet = [];
for p = penetration_depth_wet
    max_wet = [max_wet; units*p];
end

max_agar = [];
for p = penetration_depth_agar
    max_agar = [max_agar; units*p];
end

figure(1); plot(max_dry)
hold on;
plot(max_wet)
plot(max_agar);
grid on
legend('dry soil','wet soil','agar?')
title('Maximum depth of backscatter tag vs center frequency')
xlabel('Center frequency (Ghz)')
ylabel('Depth (meters)')
