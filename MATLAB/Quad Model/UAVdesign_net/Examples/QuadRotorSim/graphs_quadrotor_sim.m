close all;

t = out_time;
state = [out_xc(:,1:9) out_attitude];
% ---------------------------------------------------------
% Results
% ---------------------------------------------------------
figure;
hold on;
plot(t, state(:,1), 'k-', 'LineWidth', 2);
plot(t, state(:,2), 'b--', 'LineWidth', 2);
plot(t, state(:,3), 'r:', 'LineWidth', 2);
xlabel('time(sec)');
ylabel('V_b (m/s)');
legend('V_bx', 'V_by', 'V_bz');
title('Velocities in the body frame');
grid on;

figure;
hold on;
plot(t, state(:,4)*180/pi, 'k-', 'LineWidth', 2);
plot(t, state(:,5)*180/pi, 'b--', 'LineWidth', 2);
plot(t, state(:,6)*180/pi, 'r:', 'LineWidth', 2);
xlabel('time(sec)');
ylabel('\omega_b (deg/s)');
legend('\omega_bx', '\omega_by', '\omega_bz');
title('Rotation speeds in the body frame');
grid on;

figure;
hold on;
plot3(state(:,7),state(:,8),state(:,9),'k-','LineWidth',2);
plot3(state(1,7), state(1,8), state(1,9),'ro','MarkerEdgeColor','r', 'MarkerFaceColor',[1 0 0],'MarkerSize',8);
xlabel('N');
ylabel('E');
zlabel('D');
legend('Trajectory','Start');
title('3D Trajectory in the Earth frame (m)');
grid on;
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')
view(3);
grid on;

figure;
hold on;
plot(t, state(:,7), 'k-', 'LineWidth', 2);
plot(t, state(:,8), 'b--', 'LineWidth', 2);
plot(t, state(:,9), 'r:', 'LineWidth', 2);
xlabel('time(sec)');
ylabel('P_e (m)');
legend('N', 'E', 'D');
title('Position in the Earth frame');
grid on;

figure;
hold on;
plot(t, state(:,10)*180/pi, 'k-', 'LineWidth', 2);
plot(t, state(:,11)*180/pi, 'b--', 'LineWidth', 2);
plot(t, state(:,12)*180/pi, 'r:', 'LineWidth', 2);
xlabel('time(sec)');
ylabel('angle (deg)');
legend('\phi', '\theta', '\psi');
title('Euler angles in the Earth frame');
grid on;
