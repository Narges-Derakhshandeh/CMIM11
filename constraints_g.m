function g = constraints_g(sys, q, qd, t)
%CONSTRAINTS Return constraints vector for the system
g = zeros(length(sys.joints), 1);
g_idx = 0;

for rj = sys.joints.revolute
    q1 = q(body_idx(rj.body_i));
    q2 = q(body_idx(rj.body_j));
    qp1 = qd(body_idx(rj.body_i));
    qp2 = qd(body_idx(rj.body_j));
    A1 = rot(q1(3));
    A2 = rot(q2(3));
    fi1p = qp1(3);
    fi2p = qp2(3);
    g(g_idx + (1:2)) = A1 * rj.s1 .* fi1p ^ 2 - A2 * rj.s2 .* fi2p ^ 2;
    g_idx = g_idx + 2;
end

g_idx = g_idx + length(sys.joints.simple);

for dj = sys.joints.driving
    g(g_idx + 1) = - dj.cfun_dtt(t);
    g_idx = g_idx + 1;
end
g_idx = g_idx + length(sys.joints.translational);