function [Cset Rset] = ExtractCameraPose(E) 
% (INPUT) E: essential matrix
% (OUTPUT) Cset and Rset: four configurations of camera centers and rotations, 
%          i.e., Cset{i}=Ci and Rset{i}=Ri.

[u,d,v]=svd(E);
w1 = [0 -1 0; 
    1 0 0; 
    0 0 1];
w2 =[0 1 0;
    -1 0 0;
    0 0 1];
t1=u(:,3);
t2=-u(:,3);

Rset{1}=u*w1*v';
Rset{2}=u*w1*v';
Rset{3}=u*w2*v';
Rset{4}=u*w2*v';
Cset{1}=-Rset{1}'*t1;
Cset{2}=-Rset{2}'*t2;
Cset{3}=-Rset{3}'*t1;
Cset{4}=-Rset{4}'*t2;

if det(Rset{1}) < 0
    Rset{1} = -Rset{1}; 
    Cset{1} = -Cset{1};
end
if det(Rset{2}) < 0
    Rset{2} = -Rset{2}; 
    Cset{2} = -Cset{2};
end
if det(Rset{3}) < 0 
    Rset{3} = -Rset{3};
    Cset{3} = -Cset{3};
end
if det(Rset{4}) < 0 
    Rset{4} = -Rset{4};
    Cset{4} = -Cset{4};
end
end