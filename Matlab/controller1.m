function out = controller1(qd)
global params
% position controller params
Kp = [15;15;30];
Kd = [12;12;10];

% attitude controller params
KpM = ones(3,1)*3000;
KdM = ones(3,1)*300;

acc_des = qd.acc_des + Kd.*(qd.vel_des - qd.vel) + Kp.*(qd.pos_des - qd.pos);

% Desired roll, pitch and yaw
phi_des = 1/params.grav * (acc_des(1)*sin(qd.yaw_des) - acc_des(2)*cos(qd.yaw_des));
theta_des = 1/params.grav * (acc_des(1)*cos(qd.yaw_des) + acc_des(2)*sin(qd.yaw_des));
psi_des = qd.yaw_des;

euler_des = [phi_des;theta_des;psi_des];
pqr_des = [0;0; qd.yawdot_des];
% Thurst
qd.acc_des(3);
F  = params.m*(params.grav + acc_des(3));
% Moment
M =  params.I*(KdM.*(pqr_des - qd.omega) + KpM.*(euler_des - qd.euler));
out = [F, M];