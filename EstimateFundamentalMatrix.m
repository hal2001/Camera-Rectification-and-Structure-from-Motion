function [F0,F]= EstimateFundamentalMatrix(x1, x2)
% (INPUT) x1 and x2: 8*2 matrices where each row represents a correspondence.
% (OUTPUT) F0: 3*3 matrix.
% (OUTPUT) F: 3*3 matrix with rank 2.

A = [];
n = size(x1,1);
for i = 1 : n
A =[A; 
    x1(i,1)*x2(i,1) x1(i,2)*x2(i,1) x2(i,1) ...
    x1(i,1)*x2(i,2) x1(i,2)*x2(i,2) x2(i,2) ...
    x1(i,1) x1(i,2) 1];
end
[U,D,V]=svd(A);
f=V(:,end);
% f = SolveHomogeneousEq(A); 
F0 = [f(1:3)'; f(4:6)'; f(7:9)'];
[u,d,v] = svd(F0); 
% F1=F;
d(3,3) = 0;
F = u*d*v';

F=F/norm(F);

end