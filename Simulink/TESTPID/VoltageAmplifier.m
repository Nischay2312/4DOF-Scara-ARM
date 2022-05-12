%------------CACL1----------------
%Experimental Data from a Data Sheet
%from the plot, I get the following values 
peak = 18.442/5; %hack 20.21 
final = 17.76/5;%hack 16.0
Overshoot_val = peak - final;
RiseTime = 4.9997*1e-6; %hack 18ms
PeakTime = 6.05*1e-6; %hack 27ms
SettleTime = 10.0*1e-6; %hack 78ms
Overshoot_pct = (peak-final)/final*100; %percentage

%-----------CALC2-----------------
%we can estimate zeta from %overshoot as
%percent_OS = e^(-zeta*pi/sqrt(1-zeta^2))
%so zeta = sqrt(ln^2(OS/FV)/(pi^2+ln^2(OS/Fv))
zeta = sqrt(log(Overshoot_val/final)^2/(pi^2+log(Overshoot_val/final)^2));
%to calculate wn = 4/(Zeta*SettleTime)
omega_n = 4/(zeta*SettleTime*1e-3);

%-----------CALC3----------------
%Wn using peaktime = pi/(beta*Tpeak)
beta = sqrt(1- zeta^2);
omega_n_tp = pi/(1e-3*PeakTime*beta);

%----------CALC4------------------
%Omega_n using RiseTime = atan(beta/zeta)/(beta*Trise)
omega_n_tr = (pi-atan(beta/zeta))/(beta*RiseTime*1e-3);

%---------Making Tranfer Function---
%based on the notes, the TF of of second order system is
% TF(O2) = wn^2/(s^2 + 2*zeta*wn*s + wn^2)
%estimate 1 (use omega_n)
num1 = [ 0 0 final*omega_n^2];
den1 = [1 2*zeta*omega_n omega_n^2];
tf1 = tf(num1, den1);

%estimate 2 (use omega_n_tp)
num2 = [ 0 0 final*omega_n_tp^2];
den2 = [1 2*zeta*omega_n_tp omega_n_tp^2];
tf2 = tf(num2, den2);

%estimate 1 (use omega_n_tr)
num3 = [ 0 0 final*omega_n_tr^2];
den3 = [1 2*zeta*omega_n_tr omega_n_tr^2];
tf3 = tf(num3, den3);

%now generate step 
%input from 0 is 16V.
testpoints = 10000;
time = (linspace(0, 150*1e-6, testpoints))';
voltage = (linspace(1, 1, testpoints))';


%compute the step response of each TF
tf1_sim = lsim(tf1, voltage, time);
tf2_sim = lsim(tf2, voltage, time);
tf3_sim = lsim(tf3, voltage, time);

%now plot all four curves
figure(3);
hold on;
grid on;
box on;
plot(time*1e3, tf1_sim, ':r', 'LineWidth', 3);
plot(time*1e3, tf2_sim, ':k', 'LineWidth', 3);
plot(time*1e3, tf3_sim, ':b', 'LineWidth', 3);
lgd = legend('Estimate #1', 'Estimate #2', 'Estimate #3', 'Location', 'southeast');
xlabel('Time (msec)');
ylabel('Output (volts)');
title('Comparision between Experimental Data and Estimated Data'); 
hold off;

%------------CALC5-------------------
%Accurate Values seen from the graph
SettleTime_accurate = 78.0;%ms
OverShoot_Accurate_pct = (20.21-16.0)*100/16.0;

%-----------CALC6----------------
%use pole() command to get poles
systemPoles = pole(tf1);
systemPoles = systemPoles';

%-------------Plot Poles--------
%use the pzmap function;
figure(4);
pzmap(tf1);
grid on;
