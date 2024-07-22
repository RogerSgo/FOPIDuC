clear all;clc; 
%% Paper 3
%   Proyecto: FOPID on uC of 32 bits
%   Autor: Roger Sarango
%   Descripcion: Discretizacion con el metodo indirecto, aproximar la FT 
%                del FOPID de FO a IO y discretizar esta FT continua.
%% Vectores de tiempo
dt = 0.001;
tc = 0:dt:1; % vector tiempo para sistema de control
tp = 0:dt:5; % vector tiempo para la planta
%% Parametros de controladores FOPID
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
%% Modelo FOPDT motor DC Amax-26 Maxon con reductor
nump = 486.8;     % Constante de la planta
denp = [0.21349 1];    % Constante de tiempo
planta = tf(nump,denp,'InputDelay', 0.005651);     % FT planta mas tiempo de retardo
sal_p = step(planta,tp)*(30/pi);
%figure(1)
%plot(tp,sal_p,'blue', 'Linewidth', 1.5);hold on;
%% Controladores FOPID
Cfopid_1 = fracpid(kp_1, ki_1, lambda_1, kd_1, mu_1);
Cfopid_2 = fracpid(kp_2, ki_2, lambda_2, kd_2, mu_2);
Cfopid_3 = fracpid(kp_3, ki_3, lambda_3, kd_3, mu_3);
Cfopid_4 = fracpid(kp_4, ki_4, lambda_4, kd_4, mu_4);
Cfopid_5 = fracpid(kp_5, ki_5, lambda_5, kd_5, mu_5);
Cfopid_9 = fracpid(kp_9, ki_9, lambda_9, kd_9, mu_9);
Cfopid_10 = fracpid(kp_10, ki_10, lambda_10, kd_10, mu_10);
%% Sistema de realimentacion negativa
a1 = Cfopid_1*planta;   % lazo abierto
b1 = a1/(1+a1);     % lazo cerrado
a2 = Cfopid_2*planta; b2 = a2/(1+a2);    
a3 = Cfopid_3*planta; b3 = a3/(1+a3);      
a4 = Cfopid_4*planta; b4 = a4/(1+a4);     
%% Respuesta al escalon del sistema de control
sal_1 = step(b1,tc)*10;
sal_2 = step(b2,tc)*10;
sal_3 = step(b3,tc)*10;
sal_4 = step(b4,tc)*10;
%% Graficas de respuesta de SCOF
figure(2)
plot(tc,sal_1,'blue', 'Linewidth', 1.5);hold on;
plot(tc,sal_2,'black', 'Linewidth', 1.5);hold on;
plot(tc,sal_3,'red', 'Linewidth', 1.5);hold on;
plot(tc,sal_4,'green', 'Linewidth', 1.5);hold on;
title('Response of FOPID Controllers');xlabel('Time(sec)');ylabel('Velocity(rpm)');
legend('ZN-NM-ISE', 'ZN-NM-IAE', 'ZN-NM-ITSE', 'ZN-NM-ITAE'); grid on
% f = gca;
% exportgraphics(f,'barchart.png','Resolution',1800)
%% Parametros temporales del sistema
ir = stepinfo(sal_1)   % Obtener informacion de la respuesta
t_est = ir.SettlingTime    % Extraer valor de tiempo de establecimiento
o_sh = ir.Overshoot    % Extraer valor de Porcentaje de Overshoot
err = 1-sal_1;     % e(t)=1-y(t)
iae = trapz(tc,abs(err));
ise = trapz(tc,err.^2);
itae = trapz(tc,tc'.*abs(err));
itse = trapz(tc,tc'.*(err.^2));
%% Aproximacion ft entera kp,ki,kd,lam,u
ftcrone = nipid(kp_4,kd_4,mu_4,ki_4,lambda_4,[0.1 1000],6,'crone')
ftmatsuda = nipid(kp_4,kd_4,mu_4,ki_4,lambda_4,[0.1 1000],12,'matsuda')% inicial [0.00001 5] 0.1 10000  [0.01 1000]
ftcarlson = nipid(kp_4,kd_4,mu_4,ki_4,lambda_4,[0.1 1000],2,'carlson')
%% Discretizacion
dzcrone = c2d(ftcrone, 0.001, 'zoh')
dzmatsuda = c2d(ftmatsuda, 0.001, 'zoh')
dzcarlson = c2d(ftcarlson, 0.001, 'zoh')
%% Simulacion lazo cerrado con ft entera
figure(3)
ss = ftcrone*planta; aa = ss/(1+ss);
sale = step(aa,tc);
plot(tc,sale,'blue', 'Linewidth', 1.5);hold on;
%% Analisis de fecuencia aproximacion entera y discreta
figure(4)
bode(ftcrone, 'r', dzcrone, 'r--', ftmatsuda, 'g',dzmatsuda, 'g--', ftcarlson, 'b', dzcarlson, 'b--'); grid on; 
legend('Crone','Discret Crone','Matsuda','Discret Matsuda','Carlson', 'Discret Carlson')
