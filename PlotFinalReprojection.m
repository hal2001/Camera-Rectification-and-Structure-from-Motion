n=size(Xfinal,1);
for i=1:6
    counter=0;
    Xpos=[];
    realtraj=[];
    for j=1:n
        if traj(j,(4+2*(i-1)))~=0
        counter=counter+1;
        Xpos(counter,:)=[traj(j,1) traj(j,2) traj(j,3)];
        realtraj(counter,:)=traj(j,(4+2*(i-1)):(4+2*(i-1)+1));
        end
    end
    Reprojection( i, Xpos, realtraj, K, Csetnew{i}, Rsetnew{i} )
end