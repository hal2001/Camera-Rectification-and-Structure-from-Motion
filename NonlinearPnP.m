function [C R] = NonlinearPnP(X, x, K, C0, R0)
% (INPUT) X and x: N × 3 and N × 2 matrices whose row represents correspondences 
%         between 3D and 2D points, respectively.
% (INPUT) K: intrinsic parameter
% (INPUT and OUTPUT) C and R: camera pose (C, R).


%%
q0=R2q(R0);
pos0=[q0;C0];

fun = @(pos) ReErr(X,x,K, pos);
posnew=lsqnonlin(fun, pos0);
C=posnew(5:7,:);
q=posnew(1:4,:);
q=q/norm(q);
R=q2R(q);


%%
% n=size(X,1);
% P0=K*R0*[eye(3) -C0]; 
% % X=[X,ones(n,1)]'; %4*N
% 
% % fun= @(P)(x(:,1)'-P(1,:)*X./(P(3,:)*X))+(x(:,2)'-P(1,:)*X./(P(3,:)*X));
% 
% fun= @(P) ReprojectionError(X, x, [], P, []);
% % options=optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt', 'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'iter');
% % options=optimoptions(@lsqnonlin,'Display', 'iter');
% [P, resnorm, residual, exitflag, output]=lsqnonlin(fun, P0);%, [], [], options);
% % [P, fval]=lsqnonlin(fun, P0);
% P=K\P;
% 
% [u,d,v]=svd(P(:,1:3));
% R=u*v';
% 
% t=P(:,4)/d(1,1);
% 
% if det(R)<0
%     R=-R;
%     t=-t;
% end
% 
% C=-R'*t;

end



function e= ReErr (X, x, K, pos)

N = size(X, 1);
Xh = [X, ones(N, 1)];

C=pos(5:7,:);
q=pos(1:4,:);
% q=q/norm(q);
R=q2R(q);

P=K*R*[eye(3) -C];
xph = P * Xh';
xp = [xph(1, :) ./ xph(3, :); xph(2, :) ./ xph(3, :)];
% 
% e0 = sum((x - xp') .^ 2, 2);
E = x - xp';
E = E';
e = E(:);
% show=[e0 e]

end