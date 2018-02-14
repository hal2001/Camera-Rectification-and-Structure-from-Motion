function X = LinearTriangulation(K1, K2, C1, R1, C2, R2, x1, x2)
% (INPUT) K1 and K2: 3 * 3 calibration matrices 
% (INPUT) C1 and R1: the first camera pose 
% (INPUT) C2 and R2: the second camera pose
% (INPUT) x1 and x2: two N * 2 matrices whose row represents correspondence 
%         between the first and second images where N is 
%         the number of correspondences.
% (OUTPUT) X: N * 3 matrix whose row represents 3D triangulated point.

P1=K1*R1*[eye(3) -C1];
P2=K2*R2*[eye(3) -C2];


n=size(x1,1);
x1=[x1 ones(n,1)];
x2=[x2 ones(n,1)];

A=[];
for i=1:n
    A=[Vec2Skew(x1(i,:))*P1;
       Vec2Skew(x2(i,:))*P2];
   [u,d,v]=svd(A);
   X(:,i)=v(:,end)/v(end,end);

end

   X=X';
   X(:,end)=[];

end