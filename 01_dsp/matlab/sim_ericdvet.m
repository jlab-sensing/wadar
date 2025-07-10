% @file     link_budget_ericdvet
%
% Inherited from link_budget.m, for my thesis

clear all; close all; clc;

% r = [4 5 6 7 8];
% r = r ./ 3.2808399;
% 
% P_t = 10^(-14/10);
% G_t = 10^(4/10);
% G_r = 10^(4/10);
% G_a = 19 + 22;  % in dB
% G_a = 10^(G_a/10);
% L_n = 10^(-6/10);
% G_i = 128;
% 
% c = physconst('LightSpeed');
% f = 2.0*10^9; % Center frequency
% lambda= c/f;
% 
% sigma = pi * ((12/2)/39.3700787)^2;
% 
% P_r_red = (P_t * G_t * G_r * G_a * L_n * G_i * lambda^2)./((4*pi)^3*r.^4) * sigma;
% P_r_red = pow2db(P_r_red);
% 
% G_a = 20 + 22;  % in dB
% G_a = 10^(G_a/10);
% P_r_yellow = (P_t * G_t * G_r * G_a * L_n * G_i * lambda^2)./((4*pi)^3*r.^4) * sigma;
% P_r_yellow = pow2db(P_r_yellow);
% 
% P_r_red_measured = [-13.0852 -16.3073 -19.3694 -20.0256 -24.0386];
% 
% figure('Color', 'w');
% hold on;
% 
% % scatter(r, P_r_red, 70, 'r', 'filled', 'DisplayName', 'P_R measured (original radar)');
% plot(r, P_r_red, '--or', 'LineWidth', 1.5, 'DisplayName', 'P_R predicted (original radar)');
% plot(r, P_r_red_measured, '-or', 'LineWidth', 1.5, 'DisplayName', 'P_R measured (original radar)');
% 
% % scatter(r, P_r_yellow, 70, 'b', 'filled', 'DisplayName', 'P_R measured (news radar)');
% plot(r, P_r_yellow, '--ob', 'LineWidth', 1.5, 'DisplayName', 'P_R predicted (new radar)');
% 
% xlabel('Target Distance (m)', 'FontSize', 12);
% ylabel('Received signal strength (dBm)', 'FontSize', 12);
% % title('', 'FontSize', 14);
% ylim([min(P_r_yellow)-10 0]);
% grid on;
% xlim([min(r)-0.2 max(r)+0.2])
% 
% set(gca, 'FontSize', 12, 'LineWidth', 1);
% legend('Location', 'best', 'FontSize', 10, 'Box', 'off');
% set(gcf, 'Units', 'inches', 'Position', [1 1 5 3.5]);
% set(gca, 'TickDir', 'out');
