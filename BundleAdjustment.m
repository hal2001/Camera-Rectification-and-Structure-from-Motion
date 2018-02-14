function [Cset Rset Xset] = BundleAdjustment(num, Cset, Rset, K, traj, V)
% (INPUT) X: reconstructed 3D points. 
% (INPUT) K: intrinsic parameter 
% (INPUT) traj: a set of 2D trajectories 
% (INPUT) V: visibility matrix
% (INPUT and OUTPUT) C and R: camera pose (C, R).

CR=[];
% n=length(Cset);
for i=1:num
    q = R2q(Rset{i});
    CR=[CR;q;Cset{i}];
end

X0=traj(:,1:3);
n=size(X0,1);
X0=X0';
M0=[CR; X0(:)];

    fun= @(M) ReErr(num, n, M, K, traj, V);
    options=optimoptions(@lsqnonlin,'Display', 'iter');
    Mnew=lsqnonlin(fun, M0, [], [], options);

for i=1:num
    q=Mnew((7*(i-1)+1):(7*(i-1)+4));
    R=q2r(q);
    R=R/norm(R);
    C=M((7*(i-1)+5):(7*i)); 
    Rset{i}=R;
    Cset{i}=C;
end

Xnew=M(7*num+1:end);
Xset=reshape(Xnew, [], 3);

end


function e= ReErr(num, n, M, K, traj, V)
e=[];
X=M(7*num+1:end);
X=reshape(X, [], 3);

m=size(X,1);
X=[X ones(m,1)];

for i=1:num
    q=M((7*(i-1)+1):(7*(i-1)+4));
    R=q2R(q);
%     R=R/norm(R);
    C=M((7*(i-1)+5):(7*i));
    
    P=K*R*[eye(3) -C];
    
    x=[];
    xph=[];
    for j=1:n
        if V(i,j)~=0
            x = [x;traj(j,(4+2*(i-1)):(4+2*(i-1)+1))];
            xph = [xph, P * X(j,:)'];
        end
    end
    xp = [xph(1, :) ./ xph(3, :); xph(2, :) ./ xph(3, :)];
    E = x - xp';
    E = E';
    e = [e; E(:)];

end
end