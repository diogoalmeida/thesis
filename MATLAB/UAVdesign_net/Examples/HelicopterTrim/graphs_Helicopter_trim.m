close all;

N = in_N;
ar = out_advance_ratio;
rc = -out_xc_trim(:,11) * 60;
t = ar;
xl = 'Advance Ratio';
% t = rc;
% xl = 'Rate of Climb (fpm)';
% ---------------------------------------------------------
% Results
% ---------------------------------------------------------
figure;
subplot(2,1,1);
hold on;
plot(t, out_cmd_trim(:,1)*180/pi, 'k-', 'LineWidth', 2);
plot(t, out_cmd_trim(:,2)*180/pi, 'b--', 'LineWidth', 2);
plot(t, out_cmd_trim(:,3)*180/pi, 'r:', 'LineWidth', 2);
plot(t, out_cmd_trim(:,4)*180/pi, 'm-', 'LineWidth', 1);
xlabel(xl);
ylabel('Control Angles (deg)');
legend('Coll.', 'A_1', 'B_1', 'Pedal');
title('Control Angles');
grid on;

subplot(2,1,2);
hold on;
plot(t, out_xc_trim(:,18)*180/pi, 'k-', 'LineWidth', 2);
plot(t, out_xc_trim(:,19)*180/pi, 'b--', 'LineWidth', 2);
plot(t, out_xc_trim(:,20)*180/pi, 'r:', 'LineWidth', 2);
xlabel(xl);
ylabel('Euler Angles (deg)');
legend('\psi', '\theta', '\phi');
title('Euler Angles');
grid on;

figure;
subplot(2,1,1);
hold on;
plot(t, out_xc_trim(:,21)*180/pi, 'k-', 'LineWidth', 2);
plot(t, out_xc_trim(:,22)*180/pi, 'b--', 'LineWidth', 2);
plot(t, out_xc_trim(:,23)*180/pi, 'r:', 'LineWidth', 2);
xlabel(xl);
ylabel('M/R Flapping Angles (deg)');
legend('\beta_0', 'a_1', 'b_1');
title('M/R Flapping Angles');
grid on;

subplot(2,1,2);
hold on;
plot(t, out_xc_trim(:,24)*180/pi, 'k-', 'LineWidth', 2);
plot(t, out_xc_trim(:,25)*180/pi, 'b--', 'LineWidth', 2);
plot(t, out_xc_trim(:,26)*180/pi, 'r:', 'LineWidth', 2);
xlabel(xl);
ylabel('T/R Flapping Angles (deg)');
legend('\beta_0', 'a_1', 'b_1');
title('T/R Flapping Angles');
grid on;

figure;
MR_P_HP = in_omega_MR * out_TQ_rot_trim(:,2) / 550;
TR_P_HP = in_omega_TR * out_TQ_rot_trim(:,4) / 550;
hold on;
plot(t, MR_P_HP, 'k-', 'LineWidth', 2);
plot(t, TR_P_HP, 'b--', 'LineWidth', 2);
plot(t, MR_P_HP + TR_P_HP, 'r:', 'LineWidth', 2);
xlabel(xl);
ylabel('Power (HP)');
legend('M/R', 'T/R', 'Total');
title('Power Required');
grid on;


delta0 = (1-0.000006875*in_pressure_altitude)^5.256;
pressure = 2116.217 * delta0 * 0.006944;
theta_act = (in_outside_air_temp + 459.67) / (59 + 459.67);
sigmap = delta0 / theta_act;
rho = sigmap * 0.002378;
environment = [rho; sigmap; pressure; delta0; in_outside_air_temp; theta_act];


