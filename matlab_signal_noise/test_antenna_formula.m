% close all;
clear all;

freq = 868*10^6;
c = 299792458;
lambda = c/freq;
omega = 2*pi*freq;
L_half = lambda/2;
L_full = lambda;
L_quarter = lambda/4;
L_our_antenna = lambda/8;
beta = 2*pi/L_half;
theta = 0:0.01:2*pi;
for i=1: length(theta)
%     E_theta_quarter(i) = 1i * exp(1i * omega * (1)) * (cos(cos(theta(i)) * beta * L_quarter / 2) - cos(beta * L_quarter / 2)) / sin(theta(i));
%     E_theta_half(i) = 1i * exp(-1 * 1i * omega * (1)) * (cos(cos(theta(i)) * beta * L_half / 2) - cos(beta * L_half / 2)) / sin(theta(i));
%     E_theta_full(i) = 1i * exp(1i * omega * (1)) * (cos(cos(theta(i)) * beta * L_full / 2) - cos(beta * L_full / 2)) / sin(theta(i));
    E_our_antenna(i) = 1i * exp(1i * omega * (1)) * (cos(cos(theta(i)) * beta * L_our_antenna / 2) - cos(beta * L_our_antenna / 2)) / sin(theta(i));
end
E_our_antenna_norm = E_our_antenna / max(E_our_antenna);

% % plots the three types of antenna, ours is a half (16cm)
% figure();
% polarplot(theta-pi/2, abs(E_theta_full), 'r'); hold on
% polarplot(theta-pi/2, abs(E_theta_half), 'g'); 
% polarplot(theta-pi/2, abs(E_theta_quarter), 'b');
% legend('Full', 'Half', 'Quarter');
% title('Radiation pattern of vertical dipole antenna');
% 
% % plot as function of distance
% figure();
% polarplot(theta-pi/2, abs(E_theta_half_norm)*20, 'r'); hold on
% polarplot(theta-pi/2, abs(E_theta_half_norm)*50, 'g'); 
% polarplot(theta-pi/2, abs(E_theta_half_norm)*100, 'b');
% polarplot(theta-pi/2, abs(E_theta_half_norm)*150, 'm');
% polarplot(theta-pi/2, abs(E_theta_half_norm)*200, 'c');
% legend('20m', '50m', '100m', '150m', '200m');
% title('Radiation pattern of vertical dipole antenna');
% 
% % plot dB loss
% figure();
% polarplot(theta-pi/2, 10*log(abs(E_theta_half_norm))); 
% title('Radiation pattern of vertical dipole antenna');

% plot loss
figure();
polarplot(theta-pi/2, abs(E_our_antenna_norm)); 
title('Radiation pattern of vertical dipole antenna');


