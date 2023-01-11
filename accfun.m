 function qdd = accfun(sys, q,Cq,C)
%         Cq = constraints_dq(sys, q);
        g = zeros(length(C), 1);
        M=mass_matrix(sys);
        A = [M, Cq'
            Cq, zeros(length(C))];
        b = [forces(sys);
            g];
        qdd_lambda = A \ b;
        qdd = qdd_lambda(1:end - length(C));
    end