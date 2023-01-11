% function [T, Q, Qd] = odeEulerCromer(sys)
% 
%  q0 = initial_coordinates(sys);
% qd0 = zeros(size(q0));
% 
% dt = sys.solver.t_step;
% n_steps = ceil(sys.solver.t_final / sys.solver.t_step) + 1;
% T = linspace(0,1,n_steps);
% 
% Q = zeros(length(q0), length(T));
% Qd = zeros(length(qd0), length(T));
% 
% % Initial condition
% Q(:, 1) = q0;
% Qd(:, 1) = qd0;
% % Step equations forward in time
% for n = 1:n_steps - 1
%   qdd = mass_matrix(sys)\ forces(sys);  
% % qdd = accfun(Q(:, n), Qd(:, n), T(n));
%     Qd(:, n+1) = Qd(:, n) + dt*qdd;
%     Q(:, n+1) = Q(:, n) + dt*Qd(:, n+1);
% end
function [T, Q, Qd] = odeEulerCromer(sys)

q0 = initial_coordinates(sys);
qd0 = zeros(size(q0));

C = constraints(sys, q0, 0.0);
nC = length(C);

if norm(C) > 1e-12
    warning("Initial constraint norm is too large - expect incorrect results!")
end

M = mass_matrix(sys);
f = forces(sys);

dt = sys.solver.t_step;
n_steps = ceil(sys.solver.t_final / sys.solver.t_step) + 1;
T = linspace(0, sys.solver.t_final, n_steps);

Q = zeros(length(q0), length(T));
Qd = zeros(length(qd0), length(T));

% Initial condition
Q(:, 1) = q0;
Qd(:, 1) = qd0;


% qdd = M \ f;

g = zeros(nC, 1);

   
% Step equations forward in time
for n = 1:n_steps - 1
%   qdd = mass_matrix(sys)\ forces(sys); 
qdd = accfun(Q(:, n));
% while n<100
%   
%     Qd(:, n+1) = Qd(:, n) - dt*qdd;
%     Q(:, n+1) = Q(:, n) - dt*Qd(:, n+1)+(1/2)*qdd*(dt^2);
% end
Qd(:, n+1) = Qd(:, n) + dt*qdd;
    Q(:, n+1) = Q(:, n) + dt*Qd(:, n+1)+(1/2)*qdd*(dt^2);
end

 function qdd = accfun(q)
        Cq = constraints_dq(sys, q);
        A = [M, Cq'
            Cq, zeros(nC)];
        b = [f;
            g];
        qdd_lambda = A \ b;
        qdd = qdd_lambda(1:end - nC);
 end
end