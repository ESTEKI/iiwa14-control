% Calculatin the P matrix in A'P+PA = -Q for Robust Inverse Dynamic
%2DoF robot
function [P,B] = P_calculateFunc(KP,KD,Q_const)
DoF = size(KP,1);
dim = 2*DoF;
P_sym= sym("p",[dim,dim]);
P_sym = tril(P_sym,0) + tril(P_sym,-1).';% Sym. matrix
% KP = eye(DoF,DoF)*100;
% KD = eye(DoF,DoF) *50;
A = [zeros(DoF,DoF) eye(DoF,DoF) ; -KP -KD];
B = [zeros(DoF,DoF); eye(DoF,DoF)] ;
AP = A'*P_sym+P_sym*A ;
Q(1:dim,1:dim) = Q_const; % Choosing Q > 0
% Pvector= reshape(P_sym,1,[]);
eqns = AP+Q;
sol = solve(eqns);
P = zeros(dim,dim);
for ii = 1:dim 
    for jj =1:dim
        P(ii,jj) = double (sol.(string( P_sym(ii,jj) )));
    end
end

end