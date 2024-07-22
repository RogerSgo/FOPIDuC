clear all; clc;
%% Discertizacion FOPID para FOPDT aproximado de un proceso real (Motor DC)
tc = 0:0.001:0.1;
%% Modelo seleccionado: PID Tuner
num = 2920.8;    % K
den = [0.21349 1];  % T
P = tf(num,den,'InputDelay', 0.005651);  % L
%% Parametros del controlador
kp_1 = 0.015822; ki_1 = 7.0513; lambda_1 = 0.79262; kd_1 = 0.0018036; mu_1 = 0.80103; %k = 486.8;ise
kp_2 = 0.011452; ki_2 = 6.1209; lambda_2 = 0.79583; kd_2 = 0.0018757; mu_2 = 0.79495; % iae
kp_3 = 0.0041511; ki_3 = 8.8532; lambda_3 = 0.7193; kd_3 = 0.0035656; mu_3 = 0.70639; % itse
kp_4 = 0.020804; ki_4 = 5.5961; lambda_4 = 0.69121; kd_4 = 0.0030145; mu_4 = 0.72609; % itae

kp_5 = 0.0022118; ki_5 = 0.2364; lambda_5 = 0.50024; kd_5 = 0.0045321; mu_5 = 0.49986; %k = 2920.8;
kp_6 = 0.00015978; ki_6 = 0.24767; lambda_6 = 0.50022; kd_6 = 0.00448146; mu_6 = 0.50019;
kp_7 = 0.00037532; ki_7 = 0.25388; lambda_7 = 0.5002; kd_7 = 0.0045317; mu_7 = 0.50024;
kp_8 = 2.3745e-06; ki_8 = 0.25803; lambda_8 = 0.5002; kd_8 = 0.004404; mu_8 = 0.49989;

kp_9 = 0.023439; ki_9 = 0.079007; lambda_9 = 0.7; kd_9 = 0; mu_9 = 0; % FOPI F-MIGO 
kp_10 = 0.087609; ki_10 = 14.8144; lambda_10 = 0.61811; kd_10 = 0.00048249; mu_10 = 0.54561; % FOPID con reductor en K
kp_11 = 499.2228; ki_11 = 1.3155; lambda_11 = 0.981; kd_11 = 2.244; mu_11 = 0.52; % FOPID con reductor en K
%% Lazo cerrado
czn = fracpid(kp_4,ki_4,lambda_4,kd_4,mu_4)
la = czn*P;   % lazo abierto
lc = la/(1+la);     % lazo cerrado
sal = step(lc,tc);
figure (5)
plot(tc,sal,'blue', 'Linewidth', 1.5);hold on;grid on
title('Control system with discrete FOPID controller');xlabel('time(ms)');ylabel('Velocity(rpm)');
%% Discretizacion de los ordenes de integracion y diferenciacion
ofi = dfod1(3,0.001,1/7,-lambda_4);     % Filtro IIR, orden integral
ofd = dfod1(3,0.001,1/7, mu_4);   % Filtro IIR, orden diferencial
%oo = dfod1(3,0.001,1/7,0.5)  %pdf
%% cambiar de -lambda a lambda y de ki*(ofi) a ki*(1/ofi)
%% Frecuencia
%bode(ofi)
%% Controlador FOPID, metodo discreto
C = kp_4 + ki_4*(ofi) + kd_4*ofd
%% la
% dzP = c2d(P, 0.001, 'zoh');
% la = C*dzP; lc = la/(1+la);
% y = step(la,tc);
% plot(tc,y,'blue','Linewidth',1.5); hold on;

%%asd
%figure(1)
% bode(czn)
% figure(2)
%bode(C)