function [p1, p2] = GetInliersRANSAC(points1, points2, error, k)
% a1=points1.Location;
% a2=points2.Location;
a1=points1;
a2=points2;

m0=0;
m=size(a1,1);

counter=0;

x1=[a1,ones(m,1)];
x2=[a2,ones(m,1)];
% dis=zeros(m,1);
e=zeros(m,1);

A=[[x1(:,1), x1(:,1), x1(:,1)].*x2,...
    [x1(:,2), x1(:,2), x1(:,2)].*x2,...
    ones(m,3).*x2];

for i=1:k
    index=randperm(m,8);
    x1_rand=x1(index,:);
    x2_rand=x2(index,:);
    F=EstimateFundamentalMatrix(x1_rand,x2_rand);
%     emat=x2*F*x1';
%     for j=1:m
%     l=F*x1(j,:)'; %3*1
%     dis(j)=x2(j,:)*l/norm(l(1:2)); 
%        e(j)=emat(j,j);
%     end
e=A*F(:);

    ind=find(abs(e)<error);
%     ind=find(abs(dis)<error);

    if size(ind,1)>=m0
        counter=counter+1;
        in=ind;
        m0=size(ind,1);
        ratio(counter)=m0/m;
    end
    
end
ratio=ratio';
p1=a1(in,:);
p2=a2(in,:);

end