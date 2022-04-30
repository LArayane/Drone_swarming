function out  = quadEOM_sim(in)
% QUADEOM Wrapper function for solving quadrotor equation of motion
params.mass = 0.030;  
params.grav = 9.81;  
params.I = [1.43e-5, 0, 0; 0, 1.43e-5, 0; 0, 0, 2.89e-5];
params.arm_length = 0.046; 
params.invI = inv(params.I);
params.maxangle = 40*pi/180;
params.maxF     = 2.5*params.mass*params.grav;  
params.minF     = 0.05*params.mass*params.grav; 

s = in(1:13);
desired_state = in(14:24);

% convert state to quad stuct for control
%current state
qd.pos = s(1:3);
qd.vel = s(4:6);
Rot = QuatToRot(s(7:10)');
[phi, theta, yaw] = RotToRPY_ZXY(Rot);
qd.euler = [phi; theta; yaw];
qd.omega = s(11:13);
qd.pos_des      = desired_state(1:3);
qd.vel_des      = desired_state(4:6);
qd.acc_des      = desired_state(7:9);
qd.yaw_des      = desired_state(10);
qd.yawdot_des   = desired_state(11);

% get control outputs
%[F, M, trpy, drpy] = controlhandle(qd, t, qn, params);
%hhh = controller1(qd);

Kp = [15;15;30];
Kd = [12;12;10];

% attitude controller params
KpM = ones(3,1)*3000;
KdM = ones(3,1)*300;

acc_des = qd.acc_des + Kd.*(qd.vel_des - qd.vel) + Kp.*(qd.pos_des - qd.pos);

% Desired roll, pitch and yaw
phi_des = 1/(params.grav)*(acc_des(1)*sin(qd.yaw_des) - acc_des(2)*cos(qd.yaw_des));
theta_des = 1/(params.grav) * (acc_des(1)*cos(qd.yaw_des) + acc_des(2)*sin(qd.yaw_des));
psi_des = qd.yaw_des;

euler_des = [phi_des;theta_des;psi_des];
pqr_des = [0;0; qd.yawdot_des];
% Thurst
qd.acc_des(3);
F  = params.mass*(params.grav + acc_des(3));
% Moment
M =  params.I*(KdM.*(pqr_des - qd.omega) + KpM.*(euler_des - qd.euler));
F_no_limit = F;
M_no_limit = M;

%************ EQUATIONS OF MOTION ************************
% Limit the force and moments due to actuator limits
A = [0.25,                      0, -0.5/params.arm_length;
     0.25,  0.5/params.arm_length,                      0;
     0.25,                      0,  0.5/params.arm_length;
     0.25, -0.5/params.arm_length,                      0];

prop_thrusts = A*[F;M(1:2)]; % Not using moment about Z-axis for limits
prop_thrusts_clamped = max(min(prop_thrusts, params.maxF/4), params.minF/4);

B = [                 1,                 1,                 1,                  1;
                      0, params.arm_length,                 0, -params.arm_length;
     -params.arm_length,                 0, params.arm_length,                 0];
F = B(1,:)*prop_thrusts_clamped;
M = [B(2:3,:)*prop_thrusts_clamped; M(3)];

% Assign states
x = s(1); y = s(2); z = s(3);
xdot = s(4); ydot = s(5); zdot = s(6);
qW = s(7); qX = s(8); qY = s(9); qZ = s(10);
p = s(11); q = s(12); r = s(13);
quat = [qW; qX; qY; qZ];
bRw = QuatToRot(quat);
wRb = bRw';

% Acceleration
accel = 1 / params.mass * (wRb * [0; 0; F] - [0; 0; params.mass * params.grav]);

% Angular velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2);
qdot = -1/2*[0, -p, -q, -r;...
             p,  0, -r,  q;...
             q,  r,  0, -p;...
             r, -q,  p,  0] * quat + K_quat*quaterror * quat;

% Angular acceleration
omega = [p;q;r];
pqrdot   = params.invI * (M - cross(omega, params.I*omega));

% Assemble sdot
sdot = zeros(13,1);
sdot(1)  = xdot; sdot(2)  = ydot; sdot(3)  = zdot;
sdot(4)  = accel(1); sdot(5)  = accel(2); sdot(6)  = accel(3);
sdot(7)  = qdot(1); sdot(8)  = qdot(2); sdot(9)  = qdot(3); sdot(10) = qdot(4);
sdot(11) = pqrdot(1); sdot(12) = pqrdot(2); sdot(13) = pqrdot(3);
out = [sdot; qd.euler; euler_des; F_no_limit; M_no_limit; F; M];
