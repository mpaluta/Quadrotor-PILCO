clear all
close all
clc
%%
settings_q;
S0 = zeros(8,1);
load('20Hz');
plotting.verbosity=0;

%% test 1, Initial Control Outputs
% %figure()
% pitch = [];
% pitchingM = [];
% rollingM = [];
% roll = [];
% p = [];
% q = [];
% r = [];
% yawingM =[];
% for i = -1:.1:1;
%     %mu0 = [.5*i 0 0 0 0 0 0 0]; % rolled
%     %mu0 = [0 .5*i 0 0 0 0 0 0]; % pitched
%     mu0 = [0 0 .5*i 0 0 0 0 0]; % initial roll rate (p)
%     %mu0 = [0 0 0 .5*i 0 0 0 0]; % initial pitch rate (q)
%     %mu0 = [0 0 0 0 .5*i 0 0 0]; % initial yaw rate (r)
%     s = mu0'; sa = gTrig(s, zeros(length(s)), angi); s = [s; sa];
%     u(1,:) = policy.fcn(policy,s(poli),zeros(length(poli)));
%     My = (u(1,1)-u(1,2)-u(1,3)+u(1,4))*.06/sqrt(2); % Pitching moment
%     Mx = (-u(1,1)-u(1,2)+u(1,3)+u(1,4))*.06/sqrt(2); % Rolling moment
%     Mz = (u(1,1)-u(1,2)+u(1,3)-u(1,4)); % Yawing moment
%     pitch = [pitch mu0(2)];
%     pitchingM = [pitchingM My];
%     rollingM = [rollingM Mx];
%     roll = [roll mu0(1)];
%     yawingM = [yawingM Mz];
%     p = [p mu0(3)];
%     q = [q mu0(4)];
%     r = [r mu0(5)];
% end
% plot(p,rollingM,'LineWidth',2); hold on; grid on
% 
% title('Applied moment in state with initial roll rate (Time t = 0)')
% xlabel('p (rad/s)')
% ylabel('Moment applied (N*m)')

%% test 2, Applying Controller
mu0 = [0 0 0 .2 0 0 0 0];
endtime = 5;
H = endtime/dt;
time = 0:dt:dt*H;
applyController;

%%
figure
hold all
for i = 1:7
    plot(time,[mu0(i) xx(:,i)'], '.-')
end
legend('roll','pitch','p','q','r','u','v','Location','SouthEast')
title('State evolution over time')
xlabel('Time (s)')
