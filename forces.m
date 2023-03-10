function f = forces(sys)
    f = zeros(3 * length(sys.bodies), 1);
    for ii = 2:length(sys.bodies)
        f(body_idx(ii)) = [sys.bodies(ii).mass * sys.gravity; 0];
    end

%     function f = force_vector(grav, sforce, bodies, q)
% 
% b_len = length(bodies);
% 
% f = zeros(b_len * 3, 1);
% 
% for i = 1:b_len
%     b = bodies(i);
%     f(body_idx(i)) = [b.m * grav; 0];
% end
% 
% for sf = sforce
%     idx = body_idx(sf.i);
%     phi_i = q(idx(3));
%     f(idx) = f(idx) + [sf.f; (omega * rot(phi_i) * sf.u_i)' * sf.f];
% end