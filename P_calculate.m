% Calculatin the P matrix in A'P+PA = -Q for Robust Inverse Dynamic

P_sym= sym("p",[14,14]);
P_sym = tril(P_sym,0) + tril(P_sym,-1).';
A = [zeros(7,7) eye(7,7) ; -KP -KD];
B = [zeros(7,7); eye(7,7)] ;
AP = A'*P_sym+P_sym*A ;
Q(1:14,1:14) = 10; % Choosing Q > 0
Pvector= reshape(P_sym,1,[]);
eqns = AP+Q;
sol = solve(eqns);
P = zeros(14,14);
for ii = 1:14 
    for jj =1:14
        P(ii,jj) = double (sol.(string( P_sym(ii,jj) )));
    end
end