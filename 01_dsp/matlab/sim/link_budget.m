% @file     link_budget.m
%
% Parameter specs can be derived from Newtonian physics 
% @date     29 May 2025
% @version  1.0.0
d_t = [0.5 1.0 1.5];
d_r = [1 2 3 4 5 6 7 8 9 10];
L_r = 1.0;% 0.54;
f = 2.0*10^9;   % Hz.
c = 2.99*10^8;
gamma = 0.66;
P_T = 0.0398;   % W.
G_T = db2pow(2.5);
G_R= db2pow(2.5);
G_t = db2pow(8);
lambda= c/f;    % m.
omega = (lambda^2*G_t^2*gamma^2)/(4*pi); %pi*(0.305/2)^2;
G_i = 128;      % Integration gain.
P_R1 = [];
t = d_t(1);
for d = d_r
        % Silt loam @ 0.5 = 30+j4, silt loam @ 0.07 = 4.5 + j0.5;
        % Clay loam @ 0.5 = 26 + j4.5
        L_p = db2pow(-1*8.86*t*((2*pi*f)/c) * sqrt((30/2)*(sqrt(1+(4.5/30)^2)-1))); %36dB TODO: Should this be db2mag??
        P_R1 = [P_R1 db((P_T*G_T*G_R*L_p*L_r*G_i*db2pow(19-6+23)*lambda^2*omega)/((4*pi)^3*(d^4)),'power')];
end
P_R1

P_R2 = [];
t = d_t(2);
for d = d_r
        % Silt loam @ 0.5 = 30+j4, silt loam @ 0.07 = 4.5 + j0.5;
        % Clay loam @ 0.5 = 26 + j4.5.
        L_p = db2pow(-1*8.86*t*((2*pi*f)/c) * sqrt((30/2)*(sqrt(1+(4.5/30)^2)-1))); %36dB TODO: Should this be db2mag??
        P_R2 = [P_R2 db((P_T*G_T*G_R*L_p*L_r*G_i*db2pow(19-6+23)*lambda^2*omega)/((4*pi)^3*(d^4)),'power')];
end
P_R2

P_R3 = [];
t = d_t(3);
for d = d_r
        % Silt loam @ 0.5 = 30+j4, silt loam @ 0.07 = 4.5 + j0.5;
        % Clay loam @ 0.5 = 26 + j4.5
        L_p = db2pow(-1*8.86*t*((2*pi*f)/c) * sqrt((30/2)*(sqrt(1+(4.5/30)^2)-1))); %36dB TODO: Should this be db2mag??
        P_R3 = [P_R3 db((P_T*G_T*G_R*L_p*L_r*G_i*db2pow(19-6+23)*lambda^2*omega)/((4*pi)^3*(d^4)),'power')];
end
P_R3

plot(d_r,P_R1, '-.m.','MarkerSize',10,'DisplayName','P_R for tag at 0.5m')
hold on
plot(d_r,P_R2, '-c.','MarkerSize',10,'DisplayName','P_R for tag at 1.0m')
plot(d_r,P_R3, '--k.','MarkerSize',10,'DisplayName','P_R for tag at 1.5m')
yline(-95,':r','DisplayName','Radar max sensitivity');
% Sphere calibration results.
%plot(d_r,[-14.42,-18.69, -24.71,-26.98,-30.73], '--r.','MarkerSize',10,'DisplayName','P_R measured')
% Tag calibration results.
%plot(d_r,[-17.54,-21.88,-19.51,-23.46,-29.62], '--r.','MarkerSize',10,'DisplayName','P_R measured, \Gamma_{max}')
%plot(d_r,[-20.54,-24.06,-21.41,-24.71,-31.42], '-.m.','MarkerSize',10,'DisplayName','P_R measured, \Gamma_{min}')

ylim([-100 0])
ax = gca;
ax.YAxis.FontSize = 14; % For y-axis.
ax.XAxis.FontSize = 14; % For y-axis.
ylabel('Received signal strength (dBm)','FontSize',18)
xlim([d_r(1)-1 d_r(end)+1])
xlabel('Radar height (m)','FontSize',18)
grid on
legend('FontSize',16)
