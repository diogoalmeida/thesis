% Analysis variables
f=5; % capture frequency (needs to be set in QMC_conf.txt
T = 10; % time in secs
t = 1;

attitude = zeros(6,T*f);

% open connection
q = QMC;

disp('Press any key to start aquisition...');
pause();
% read data

a = tic;
while toc(a) < T
    b = tic;
    data = QMC(q);
    attitude(:,t) = data(:,3);
    t=t+1;
    
   
    pause(1/f-toc(b));
end

% close connection

QMC(q,'disconnect');

