function [X, x1, x2] = NonlinearTriangulation(K1, K2, C1, R1, C2, R2, x1, x2, X0)

% (INPUT) C1 and R1: the first camera pose 
% (INPUT) C2 and R2: the second camera pose
% (INPUT) x1 and x2: two N × 2 matrices whose row represents correspondence 
%         between the first and second images where N is the number of 
%         correspondences.
% (INPUT and OUTPUT) X: N × 3 matrix whose row represents 3D triangulated point.

% X0=[X0, zeros(size(X0,1),1)]'; %4*N 

% fun= @(X) ReprojectionError(X, x1, x2, P1, P2);
% % options=optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt', 'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'iter');
% options=optimoptions(@lsqnonlin,'Display', 'iter');
% [X, resnorm, residual, exitflag, output]=lsqnonlin(fun, X0, [], [], options);
% 
% ind=find(X(:,3)<0 && X(:,3)>100);
% X(ind,:)=[];

%%
% pos0=X0(:);
% fun = @(X) ReErr(X, x1, x2, K1, K2, C1, R1, C2, R2);
% Xnew=lsqnonlin(fun, pos0);
% X=reshape(Xnew, [], 3);
% 
% in1=find(X(:,3)<0);
% in2=find(X(:,3)>100);
% ind=[in1;in2];
% 
% X(ind,:)=[];
% x1(ind,:)=[];
% x2(ind,:)=[];

%%
P1=K1*R1*[eye(3) -C1];
P2=K2*R2*[eye(3) -C2];
X=[];
ind=[];
for i=1:size(x1,1)
    fun= @(X) ReErr(X, x1(i,:), x2(i,:), P1, P2);
%     options=optimoptions(@lsqnonlin,'Display', 'iter');
    Xnew=lsqnonlin(fun, X0(i,:));%, [], [], options);
    if Xnew(:,3)>C1(3) &&  Xnew(:,3)>C2(3) && Xnew(:,3)<100 
    X=[X; Xnew];
    else 
        ind=[ind;i];
    end
end


x1(ind,:)=[];
x2(ind,:)=[];
% X=X';
% X(:,end)=[];

% ind=find(X(:,3)<0);
% X(ind,:)=[];
% 
end


function e= ReErr (X,x1,x2,P1,P2)

X=reshape(X, [], 3);
Xh = [X, ones(size(X,1),1)];

% P1=K1*R1*[eye(3) -C1];
% P2=K2*R2*[eye(3) -C2];
xph1 = P1 * Xh';
xp1 = [xph1(1, :) / xph1(3, :); xph1(2, :) / xph1(3, :)];
xph2 = P2 * Xh';
xp2 = [xph2(1, :) / xph2(3, :); xph2(2, :) / xph2(3, :)];
e = [x1 - xp1'; x2-xp2'];
% E = E';
% e = E(:);

end


