%% Narges Derakhshandeh. Homework11
% Globally we have a complete multibody system
% It must contain bodies, joints, analysis settings
clear all; clc; close all;
sys = make_system();
%% Bodies


% What do we need to describe our body?
% location, orientation, name 

sys = add_body(sys, "ground");
sys = add_body(sys, "crank", [0.0866/2, 0.05/2], deg2rad(30),  2.0, 2.0 * 0.15^2 / 12);
sys = add_body(sys, "link", [0.2803/2, 0.05/2], -0.2527, 1.0,  0.5^2 / 12);
sys = add_body(sys, "slider", [0.2803, 0], 0.0,  0.1);

% Joints - kinematic (revolute and simple)
sys = add_joint_revolute(sys, "ground", "crank", [0; 0], [0.0; 0]);
sys = add_joint_revolute(sys, "crank", "link", [0.086; 0.05], [0.086; 0.05]);
sys = add_joint_revolute(sys, "link", "slider", [0.2803; 0], [0.2803; 0]);

%sys = add_joint_simple(sys, "slider", "y");
%sys = add_joint_simple(sys, "slider", "fi");

sys = add_translational_joint(sys, "slider", "ground", [0.2803; 0] ,[0.3;0]);
% sys = add_translational_joint(sys, "slider", "fi");
% sys = add_translational_joint(sys, "slider", "y");
% sys = add_translational_joint(sys, "slider", "fi");

sys = add_joint_simple(sys, "ground", "x");
sys = add_joint_simple(sys, "ground", "y");
sys = add_joint_simple(sys, "ground", "fi");

sys = add_joint_simple_driving(sys, "crank", "fi", ...,
    @(t) deg2rad(30) - 1.2 * t, ...
    @(t) - 1.2);

sys = set_solver_settings(sys,1, 0.01);
%t=linspace(0,20,0.001);

%% SOLVER EULER CROMER

%[T, Q, Qd] = solve_EC(sys);

%% SOLVER fsolve

%[Tf, Qf] = solve_kinematics_fsolve(sys);
[T, Q, Qd] = odeEulerCromer(sys);

%% SOLVER NR

% [T, Q, Qd] = solve_kinematics_NR(sys);

%% POSTPROCESSING
pidx = 10;
% plot(T, Qd(pidx, :),T(1:end-1), (Q(pidx, 2:end)-Q(pidx, 1:end-1))/0.001, '--')
% axis equal
%% 
q0 = initial_coordinates(sys);
qd0 = zeros(size(q0));
C = constraints(sys, q0, 0.01);
Cq = constraints_dq(sys, q0);
Ct = constraints_dt(sys, 0.01);
F = zeros(length(forces(sys)));
alpha = 10;
beta = 10;
Cp = zeros(length(q0),length(T));

for i = 1:length(T)

    Cp(:,i) = Cq * Qd(:, i) + Ct;

    LHS = [mass_matrix(sys), Cq'; Cq, zeros(length(C))];
    rhs(:, i) = vertcat(forces(sys), 9.81 - 2 * alpha * Cp(:,i) - beta^2 * C);
    qddlambda(:,i) = LHS \ rhs(:,i);
    Qdd(:,i) = qddlambda(1:length(q0),i);

end


%% Plot
figure(1)
plot(T, Q(pidx, :),"b")
xlabel('Time');
ylabel('Postion');
figure(2)
   plot(T, Qd(pidx, :),"r")
   xlabel('Time');
ylabel('Velocity');
   figure(3)
   plot( T, Qdd(pidx, :),"g")
   xlabel('Time');
ylabel('Acceleration');

