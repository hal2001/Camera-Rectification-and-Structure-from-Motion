function [C R] = PnPRANSAC(X, x, K, k, error)
% (INPUT) X and x: N × 3 and N × 2 matrices whose row represents correspondences 
%         between 3D and 2D points, respectively.
% (INPUT) K: intrinsic parameter
% (INPUT) k: iteration      error: maximum error allowed
% (OUTPUT) C and R: camera pose (C, R).
counter=0;
m0=0;
m=size(X,1);
X=[X,ones(m,1)]; %N*4
% x=[x,ones(m,1)]; %N*3

err=zeros(m,1);

for i=1:k
    index=randperm(m,6);
    X_rand=X(index,:);
    x_rand=x(index,:);
    [C0 R0]=LinearPnP(X_rand, x_rand, K);
    P=K*R0*[eye(3) -C0];
%     for j=1:m
%         err(j)=(x(j,1)-P(1,:)*X(j,:)'/(P(3,:)*X(j,:)'))^2+...
%             (x(j,2)-P(2,:)*X(j,:)'/(P(3,:)*X(j,:)'))^2;
%     end
    
%      err=(x(:,1)'-P(1,:)*X'./(P(3,:)*X')).^2+...
%             (x(:,2)'-P(2,:)*X'./(P(3,:)*X')).^2;

x_re0=P*X';
x_re=x_re0(1:2,:)./x_re0(3,:);
err=sum((x-x_re').^2,2);
        
    ind=find(abs(err)<error);
%     ind=ind';
%     ratio=size(ind,1)/m
    if size(ind,1)>=m0
        m0=size(ind,1);
        in=ind;
        counter=counter+1;
        ratio(counter)=m0/m;
    end
end

[C R]=LinearPnP(X(in,:), x(in,:), K);

end