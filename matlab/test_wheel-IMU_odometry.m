close all;clear all;clc;
global toRPM toRadSec
toRPM    = 60/(2*pi);
toRadSec = 1/toRPM;
%% differential drive model PID
dt = 0.01; % 100 Hz
t  = 0:dt:360;
r = 0.1; % 바퀴 반지름 [m] 
B  = 0.40; % 바퀴 간 거리 [m]

psi_traj  = (0.5*cos(t/2.2)-0.1*cos(t/1.3)-4.4*(cos(t/9))-(0.5-0.1-4.4));
v_traj    = 0.2*(sin(t*0.8-pi/2)+1);

x_traj = zeros(1,size(t,2)); dx_traj = zeros(1,size(t,2));
y_traj = zeros(1,size(t,2)); dy_traj = zeros(1,size(t,2));
dpsi_traj = zeros(1,size(t,2));
for i = 1:size(t,2)-1
   x_traj(i+1) = x_traj(i) + dt*v_traj(i)*cos(psi_traj(i));
   dx_traj(i)  = (x_traj(i+1)-x_traj(i))/dt;
   y_traj(i+1) = y_traj(i) + dt*v_traj(i)*sin(psi_traj(i));
   dy_traj(i)  = (y_traj(i+1)-y_traj(i))/dt;
   dpsi_traj(i) = (psi_traj(i+1)-psi_traj(i))/dt;
end

figure();
subplot(1,3,1);plot(t,v_traj);
subplot(1,3,2);plot(t,psi_traj);

figure();
plot(0,0,'bo','linewidth',2); hold on;
plot(x_traj,y_traj,'rx');
plot(x_traj(end),y_traj(end),'go','linewidth',2);
legend('start','traj.','end');

%
p_true  = [x_traj;y_traj];
v_true  = [dx_traj;dy_traj];
vel_true = sqrt(dx_traj.^2+dy_traj.^2);
th_true = psi_traj;
w_true  = [dpsi_traj];


% Generate sensor data
PULSE_PER_REV = 19*49*4;
dt_encoder    = 0.01; % 10 ms
err_encoder   = 1; % 0.5 pulse error
na = 2*pi*err_encoder/PULSE_PER_REV/dt_encoder; % rad/s 

nvb = (na*r+na*r)/2;
nwb = (na*r+na*r)/B;

RateNoiseSpectralDensity = 0.01; % deg/s/sqrt(Hz)

nwi = RateNoiseSpectralDensity/180*pi*sqrt(0.005)*2;

G  = [r/2,r/2;-r/B,r/B];
iG = G^-1;
alar_true = iG*[vel_true;w_true];

al    = alar_true(1,:) + na*randn(1,length(alar_true));
ar    = alar_true(2,:) + na*randn(1,length(alar_true));

vb = (al*r+ar*r)/2;
wb = (-al*r+ar*r)/B;
wi = w_true + nwi*randn(1,length(alar_true));

nwi = nwi*2;

Q = diag([0.05^2,0.05^2,0.2^2,(2/180*pi)^2,(0.5/180*pi)^2]);
R = diag([nvb^2,nwb^2,nwi^2]);

%% Dead reckoning (wheel rot + wheel vel)
X_dead = zeros(5,length(t));
for k = 1:length(t)-1
   Rk1 = [cos(X_dead(4,k));sin(X_dead(4,k))];
   Fk  = [eye(2,2),Rk1*dt,zeros(2,1),zeros(2,1);...
      0,0,1,0,0;...
      0,0,0,1,1*dt;...
      0,0,0,0,1];
   
   % pred
   X_dead(:,k+1) = Fk*[X_dead(1:2,k);vb(k);X_dead(4,k);wb(k)];
end

%% Dead reckoning (imu rot + wheel vel)
X_dead_imu = zeros(5,length(t));
for k = 1:length(t)-1
   Rk1 = [cos(X_dead_imu(4,k));sin(X_dead_imu(4,k))];
   Fk  = [eye(2,2),Rk1*dt,zeros(2,1),zeros(2,1);...
      0,0,1,0,0;...
      0,0,0,1,1*dt;...
      0,0,0,0,1];
   
   % pred
   X_dead_imu(:,k+1) = Fk*[X_dead_imu(1:2,k);vb(k);X_dead_imu(4,k);wi(k)];
end

%% Kalman Filter
X_est  = zeros(5,length(t));
P_pred = eye(5,5)*1;
P_est  = eye(5,5)*1;
P_diag = zeros(5,length(t));
for k = 1:length(t)-1
   Rk1 = [cos(X_est(4,k));sin(X_est(4,k))];
   Fk  = [eye(2,2),Rk1*dt,zeros(2,1),zeros(2,1);...
      0,0,1,0,0;...
      0,0,0,1,1*dt;...
      0,0,0,0,1];
   H = [0,0,1,0,0;0,0,0,0,1;0,0,0,0,1];
   zk = [vb(k);wb(k);wi(k)];
   
   % pred
   X_pred = Fk*X_est(:,k);
   P_pred = Fk*P_est*Fk.'+Q;
   
   % est
   K             = P_pred*H.'*(H*P_pred*H.'+R)^-1;
   P_est         = (eye(5)-K*H)*P_pred;
   X_est(:,k+1)  = X_pred + K*(zk-H*X_pred);
   P_diag(:,k+1) = diag(P_est);
end


figure('name','KF result');
subplot(4,1,1);
plot(t,p_true(1,:),'r--','linewidth',2); hold on;
plot(t,p_true(2,:),'c--','linewidth',2); 
plot(t,X_est(1,:),'m','linewidth',1); plot(t,X_est(2,:),'b','linewidth',1); ylabel('pos. [m]');
subplot(4,1,2);
plot(t,vel_true(1,:),'k--','linewidth',2); hold on;
plot(t,X_est(3,:),'r','linewidth',1);
ylabel('vel. [m/s]');
plot(t,vel_true(1,:)+sqrt(P_diag(3,:))*3,'k-.');
plot(t,vel_true(1,:)-sqrt(P_diag(3,:))*3,'k-.');
subplot(4,1,3);
plot(t,th_true,'k--','linewidth',2); hold on;
plot(t,X_est(4,:),'r','linewidth',1);
ylabel('angle [rad]');
subplot(4,1,4);
plot(t,w_true,'k--','linewidth',2); hold on;
plot(t,X_est(5,:),'r','linewidth',1);
ylabel('w [rad/s]');



figure('name','KF vs. wheel odometry');
subplot(4,1,1);
plot(t,p_true(1,:),'r--','linewidth',2); hold on;
plot(t,p_true(2,:),'c--','linewidth',2); 
plot(t,X_est(1,:),'m','linewidth',1);
plot(t,X_est(2,:),'b','linewidth',1); ylabel('pos. [m]');
subplot(4,1,2);
plot(t,vel_true(1,:),'k--','linewidth',2); hold on;
plot(t,X_est(3,:),'r','linewidth',1);
plot(t,X_dead(3,:),'b');
ylabel('vel. [m/s]');
subplot(4,1,3);
plot(t,th_true,'k--','linewidth',2); hold on;
plot(t,X_est(4,:),'r','linewidth',1);
plot(t,X_dead(4,:),'b');
ylabel('angle [rad]');
subplot(4,1,4);
plot(t,w_true,'k--','linewidth',2); hold on;
plot(t,X_est(5,:),'r','linewidth',1);
plot(t,X_dead(5,:),'b');
ylabel('w [rad/s]');

figure('name','2-D plot : KF vs. wheel odometer');
plot(p_true(1,:),p_true(2,:),'k'); hold on;
plot(X_est(1,:),X_est(2,:),'r');
plot(X_dead(1,:),X_dead(2,:),'b');
axis equal;

%%
dp = p_true - X_est(1:2,:);
err_kf = sqrt(dp(1,:).^2 + dp(2,:).^2);
dp = p_true - X_dead(1:2,:);
err_dr = sqrt(dp(1,:).^2 + dp(2,:).^2);
dp = p_true - X_dead_imu(1:2,:);
err_dr_imu = sqrt(dp(1,:).^2 + dp(2,:).^2);

angle_err_kf = abs(th_true - X_est(4,:));
angle_err_dr = abs(th_true - X_dead(4,:));
angle_err_dr_imu = abs(th_true - X_dead_imu(4,:));

figure();
subplot(2,1,1);
plot(t,err_kf,'r'); hold on;
plot(t,err_dr,'b');
plot(t,err_dr_imu,'k');
ylabel('pos. err. [m]'); xlabel('t [s]'); legend('KF','wheel odom','imu odom');
subplot(2,1,2);
plot(t,angle_err_kf*180/pi,'r'); hold on;
plot(t,angle_err_dr*180/pi,'b'); 
plot(t,angle_err_dr_imu*180/pi,'k');
ylabel('angle err. [deg]'); xlabel('t [s]'); legend('KF','wheel odom','imu odom');
