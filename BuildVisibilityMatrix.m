function V = BuildVisibilityMatrix(traj)
% (INPUT) traj: a matrix containing 
%               row1:3: 3D position
%               row4:5: 2D correspondence in frame 1
%               row6:7: 2D correspondence in frame 2
%               ...
% (OUTPUT) V: I × J visibility matrix

n=size(traj, 1); %# of points
m=6; %# of cameras
V=ones(m,n); 

M=zeros(n,m);
M(:,1)=traj(:,4);
M(:,2)=traj(:,6);
M(:,3)=traj(:,8);
M(:,4)=traj(:,10);
M(:,5)=traj(:,12);
M(:,6)=traj(:,14);

for i=1:n
    [~, col]=find(M(i,:)==0); %%%%%
    V(col,i)=0;
end

end