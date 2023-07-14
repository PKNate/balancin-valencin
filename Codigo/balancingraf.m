%Gráfica de control de posición
clc;
clear all;
close all;
%leyendo las muestras tomadas al sistema
fid=fopen('datos.TXT','r');
datos=fscanf(fid, '%f', [16 inf]);    % datos tiene 15 renglones
fclose(fid);
t=datos(1,:); 
phid=datos(2,:);
phi=datos(3,:);
omegadI=datos(4,:);
omegaI=datos(5,:);
omegadD=datos(6,:); 
omegaD=datos(7,:);
uI=datos(8,:);
uD=datos(9,:);
sI=datos(10,:);
sC=datos(11,:);
sD=datos(12,:);
sP=datos(13,:);
omegaP=datos(14,:);
omegadP=datos(15,:);
omegaPF=datos(16,:);

figure(1)
subplot(6,1,1)
plot(t,phid,t,phi);
set(legend('$\phi_d[rad]$','$\phi[rad]$'), 'interpreter', 'latex')
grid on;

subplot(6,1,2)
plot(t,omegadI,t,omegaI,t,omegadP,t,omegaP,t,omegaPF);
set(legend('$\omega_{dI}[\frac{rad}{s}]$','$\omega_I[\frac{rad}{s}]$','$\omega_{dP}[\frac{rad}{s}]$','$\omega_P[\frac{rad}{s}]$','$\omega_PF[\frac{rad}{s}]$'), 'interpreter', 'latex')
grid on;

subplot(6,1,3)
plot(t,uI);
set(legend('$u_I[V]$'), 'interpreter', 'latex')
grid on;

subplot(6,1,4)
plot(t,omegadD,t,omegaD,t,omegadP,t,omegaP,t,omegaPF);
set(legend('$\omega_{dD}[\frac{rad}{s}]$','$\omega_D[\frac{rad}{s}]$','$\omega_{dP}[\frac{rad}{s}]$','$\omega_P[\frac{rad}{s}]$','$\omega_PF[\frac{rad}{s}]$'), 'interpreter', 'latex')
grid on;

subplot(6,1,5)
plot(t,uD);
set(legend('$u_D[V]$'), 'interpreter', 'latex')
grid on;

subplot(6,1,6)
plot(t,sI,t,sC,t,sD,t,sP);
set(legend('$sI$','$sC$','$sD$','$sP$'), 'interpreter', 'latex')
xlabel('$t[s]$', 'interpreter', 'latex')
grid on;
