% A lot of the processes used to create the plots on my
% (Eric Vetha) thesis are here.

clear all; close all; clc;

% %% Link budget LOS
% 
% frequencies = [0.5e9, 1.0e9, 1.5e9, 2.0e9, 2.5e9];
% r = 1; % Tag depth in meters
% 
% G_T = 10^(4/10);  
% G_R = 10^(4/10);  
% 
% G_t = 8; 
% G_t = 10^(G_t / 10); 
% Gamma = 0.66; 
% 
% P_dBm = linspace(0, 100, 100); % transmit power in dBm
% P_T = 10.^(P_dBm / 10); % Convert to Watts
% 
% figure('Color', 'w'); hold on;
% colors = lines(length(frequencies));
% 
% for i = 1:length(frequencies)
%     f = frequencies(i);
%     lambda = physconst('LightSpeed') / f;
% 
%     sigma = (lambda^2 * G_t^2 / (4*pi)) * abs(Gamma)^2;
% 
%     P_R = (P_T * G_T * G_R * lambda^2 ./ ((4*pi)^3 * r^4)) * sigma;
%     P_R_dBm = pow2db(P_R);
% 
%     plot(P_dBm, P_R_dBm, '-', ...
%         'Color', colors(i,:), 'LineWidth', 1.8, ...
%         'DisplayName', sprintf('f = %.1f GHz', f/1e9));
% end
% 
% yline(-90, 'r', 'DisplayName', 'General radar sensitivity');
% 
% xlabel('Total Transmit Power P (dBm)', 'FontSize', 12);
% ylabel('Received Power P_R (dBm)', 'FontSize', 12);
% grid on;
% legend('Location', 'southeast', 'FontSize', 10, 'Box', 'off');
% set(gca, 'FontSize', 12, 'LineWidth', 1, 'TickDir', 'out');
% set(gcf, 'Units', 'inches', 'Position', [1 1 6 4]);
% ylim([-100 0])
% 
% %% Link budget NLOS
% 
% frequencies = [0.5e9, 1.0e9, 1.5e9, 2.0e9, 2.5e9]; 
% r = 0.5; 
% d_s = 0.5; 
% 
% P_dBm = linspace(0, 100, 100); 
% P_W = 10.^(P_dBm / 10); % Convert to Watts
% 
% eps_r_real = 23;    
% eps_r_imag = 4.4;   
% mu_r = 1;           % non-magnetic soil
% 
% % Preallocate
% colors = lines(length(frequencies));
% figure('Color', 'w'); hold on;
% 
% for i = 1:length(frequencies)
%     f = frequencies(i);
%     lambda = physconst('LightSpeed') / f;
% 
%     sigma = (lambda^2 * G_t^2 / (4*pi)) * abs(Gamma)^2;
% 
%     L_r = 1 - abs((sqrt(eps_r_real) - 1)/(sqrt(eps_r_real) + 1))^2;
% 
%     alpha = (2*pi / lambda) * sqrt((mu_r * eps_r_real / 2) * ...
%             (sqrt(1 + (eps_r_imag / eps_r_real)^2) - 1));
% 
%     L_p_dB = 8.868 * d_s * alpha;
%     L_p = 10^(-L_p_dB / 10);
% 
%     P_R = (P_W * G_T * G_R * L_r * L_p * lambda^2) ./ ((4*pi)^3 * r^4) * sigma;
%     P_R_dBm = pow2db(P_R);
% 
%     % Plot
%     plot(P_dBm, P_R_dBm, 'LineWidth', 1.8, 'Color', colors(i,:), ...
%         'DisplayName', sprintf('f = %.1f GHz', f/1e9));
% end
% 
% xlabel('Transmit Power (dBm)', 'FontSize', 12);
% ylabel('Received Power P_R (dBm)', 'FontSize', 12);
% grid on;
% legend('Location', 'southeast', 'FontSize', 10, 'Box', 'off');
% set(gca, 'FontSize', 12, 'LineWidth', 1, 'TickDir', 'out');
% ylim([-100 0]);
% set(gcf, 'Units', 'inches', 'Position', [1 1 6 4]);
% 
% yline(-90, 'r', 'DisplayName', 'General radar sensitivity');


%% Penetration Depth vs Frequency for Loam

% f = logspace(6, 10, 500);       % 1 MHz to 10 GHz
% c = physconst('LightSpeed');
% lambda0 = c ./ f;    
% 
% mu_r = 1; % because soil is non-magnetic?
% 
% % https://ntrs.nasa.gov/api/citations/19750018483/downloads/19750018483.pdf
% soil_types = {
%     'Loam @ 0.00 VWC ', [3, 0.01];
%     'Loam @ 0.20 VWC', [8.5, 2];
%     'Loam @ 0.40 VWC', [23, 4.4]
% };
% 
% figure('Color', 'w');
% hold on;
% grid on;
% 
% % For all soil types
% for i = 1:size(soil_types, 1)
%     name = soil_types{i,1};
%     eps_r = soil_types{i,2};
%     eps_r_real = eps_r(1);
%     eps_r_imag = eps_r(2);
% 
%     alpha = (2*pi ./ lambda0) .* sqrt( ...
%         (mu_r * eps_r_real / 2) .* ...
%         (sqrt(1 + (eps_r_imag ./ eps_r_real).^2) - 1) ...
%     );
% 
%     delta_p = 1 ./ alpha;
% 
%     plot(f / 1e6, delta_p, ...
%         'LineWidth', 1.5, ...
%         'DisplayName', name);
% end
% 
% % title('Penetration Depth vs Frequency for Loam');
% xlabel('Frequency (MHz)', 'FontSize', 12);
% ylabel('Penetration Depth \delta_p (m)', 'FontSize', 12);
% set(gca, 'FontSize', 12, 'LineWidth', 1, ...
%     'XScale', 'log', 'YScale', 'log', ...
%     'TickDir', 'out');
% % xlim([min(f), max(f)]);
% % ylim([min(delta_p), max(delta_p)]); 
% xline(0.435e9 / 1e6, 'r', 'DisplayName', "Our UWB's lower cutoff")
% 
% legend('Location', 'best', 'FontSize', 10, 'Box', 'off');
% set(gcf, 'Units', 'inches', 'Position', [1, 1, 5, 3.5]);

%% For specific frequencies

f = 250e6;
c = physconst('LightSpeed');
lambda0 = c ./ f;    

mu_r = 1; % because soil is non-magnetic?

% https://ntrs.nasa.gov/api/citations/19750018483/downloads/19750018483.pdf
soil_types = {
    'Loam @ 0.00 VWC ', [3, 0.01];
    'Loam @ 0.20 VWC', [8.5, 2];
    'Loam @ 0.40 VWC', [23, 4.4]
};

for i = 1:size(soil_types, 1)
    name = soil_types{i,1};
    eps_r = soil_types{i,2};
    eps_r_real = eps_r(1);
    eps_r_imag = eps_r(2);

    alpha = (2*pi ./ lambda0) .* sqrt( ...
        (mu_r * eps_r_real / 2) .* ...
        (sqrt(1 + (eps_r_imag ./ eps_r_real).^2) - 1) ...
    );

    delta_p = 1 ./ alpha;
    disp(name)
    disp(delta_p)
end

%% 

% perm2VWC = [-0.00123134046571023,0.400500684935833,-8.55808020871616];
% 
% c= physconst('LightSpeed');       % Speed of light (m/s)
% resolution = 0.003790984152165; % Range resolution (m)
% % resolution = linspace(0.003790984152165, 0.01, 100);
% 
% resolution2VWC(resolution, c, perm2VWC);
% 
% resolutions = linspace(0.003790984152165, 0.05, 1000);
% resolutions = logspace(-3, -1, 100);
% bandwidths = c ./ (2 .* resolutions);
% 
% resolutions_VWC = [];
% 
% for i = 1:length(resolutions)
%     resolutions_VWC(end+1) = resolution2VWC(resolutions(i), c, perm2VWC);
% end
% 
% 
% function resolution_VWC = resolution2VWC(resolution, c, perm2VWC)
%     distance = 0.05080010160020321;
% 
%     groundTruth = 25.12;
%     peakDifference = 80:144;
% 
%     t = ((peakDifference + ...
%         distance./resolution) .* resolution) ./ c; % Time of Flight (ToF) from on tag to the other (s)
%     radar_perm = ((c*t)/distance).^2;           % ToF converted to permittivity
%     VWC = polyval(perm2VWC, radar_perm);
%     resolution_VWC = mean(abs(diff(VWC)));
% end
% 
% figure('Color', 'w');
% hold on;
% grid on;
% 
% plot(resolutions, resolutions_VWC, 'HandleVisibility', 'off');
% 
% % title('title');
% xlabel('Range Resolution (m)', 'FontSize', 12);
% ylabel('VWC resolution (%)', 'FontSize', 12);
% set(gca, 'FontSize', 12, 'LineWidth', 1, ...
%     'XScale', 'log', 'YScale', 'log', ...
%     'TickDir', 'out');
% % xlim([min(f), max(f)]);
% ylim([10e-2, 10]); 
% yline(3, 'r', 'DisplayName', "Maximum VWC resolution (%)")
% 
% legend('Location', 'best', 'FontSize', 10, 'Box', 'off');
% set(gcf, 'Units', 'inches', 'Position', [1, 1, 5, 3.5]);