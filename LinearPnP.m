function [C R] = LinearPnP(X, x, K)
% (INPUT) X and x: N × 4 and N × 3 matrices whose row represents 
%        correspondences between 3D and 2D points, respectively.
% (INPUT) K: intrinsic parameter
% (OUTPUT) C and R: camera pose (C, R).

n=size(X,1);

x=[x,ones(n,1)]; 
% x=K\x'; %N*3
% x=x';
% u=u'; %3*N
% X=[X,ones(n,1)]'; 
% X=X'; %4*N

A=[];
for i=1:n 
    A=[A; Vec2Skew(x(i,:))*...
        [X(i,:) zeros(1,8);...
        zeros(1,4) X(i,:) zeros(1,4);...
        zeros(1,8) X(i,:)]];

end

% u=[x(:,1),x(:,1),x(:,1),x(:,1)];
% v=[x(:,2),x(:,2),x(:,2),x(:,2)];
% A=[X,zeros(n,4),-u.*X;
%     zeros(n,4),X,-v.*X];

[U,D,V]=svd(A);
% p=V(:,end);
% P=[p(1:4)'; p(5:8)'; p(9:12)'];

p1=V(1:4,end);
p2=V(5:8,end);
p3=V(9:12,end);
P=[p1';p2';p3'];

P=P/norm(P);
% P=K\P;

R=K\P(:,1:3);
[u,d,v]=svd(R);
R=u*v';

% [u,d,v]=svd(P(:,1:3));
% R=u*v';
% 
t=K\P(:,4)/d(1,1);

if det(R)<0
    R=-R;
    t=-t;
end
C=-R'*t;

end
    