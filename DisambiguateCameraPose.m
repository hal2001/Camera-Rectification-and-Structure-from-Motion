function [C R X0] = DisambiguateCameraPose(Cset, Rset, Xset)
% (INPUT) Cset and Rset: four configurations of camera centers and rotations 
% (INPUT) Xset: four sets of triangulated points from 
%               four camera pose configurations 
% (OUTPUT) C and R: the correct camera pose
% (OUTPUT) X0: the 3D triangulated points from the correct camera pose

num=0;
for i=1:4
    counter=0;
    n=size(Xset{i});
  for j=1:n
      x(j,:)=Xset{i}(j,:);
    if Rset{i}(3,:)*(x(j,:)'-Cset{i})>0 && x(j,3)>0
        counter=counter+1;
    end
  end
    if counter>=num
        num=counter;
        C=Cset{i};
        R=Rset{i};
        X0=Xset{i};
    end
end
end