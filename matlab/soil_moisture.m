% Script to calculate VWC from ToF and teros-12 sensor measurements
c=299792458;
resolution =  0.004; %0.003790984152165;
airBins = 215; % number of bins from the radar to the un-burried tag
airSoilBins = (240+240+239)/3.0; % number of bins from the radar to the burried tag
RAW = 3500;  %RAW teros12 measurement
d=0.055; % meters sensor is buried underground
t=((airSoilBins-airBins+d/resolution)*resolution)/c; %seconds


radar_perm = ((c*t)/d)^2 % apparent dielectric permittivity
teros_perm = ((2.887e-9)*RAW^3-(2.080e-5)*RAW^2+(5.276e-2)*RAW-43.39)^2

%TODO: adapt teros eqns to be used with Perm instead of RAW
teros_calibrated = (1.059e-9)*RAW^3 - (8.088e-6)*RAW^2 + (2.066e-2)*RAW - 1.727e1 %calibrated by us
teros_potting =   (6.771e-10)*RAW^3 - (5.105e-6)*RAW^2 + (1.302e-2)*RAW - 10.848 %from p. 8 of teros 12 manual
radar_topp = (4.3e-6)*radar_perm^3 - (5.5e-4)*radar_perm^2 + (2.92e-2)*radar_perm - 5.3e-2
teros_topp = (4.3e-6)*teros_perm^3 - (5.5e-4)*teros_perm^2 + (2.92e-2)*teros_perm - 5.3e-2