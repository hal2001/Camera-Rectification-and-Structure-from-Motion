function []=Reprojection(imgnum, X, x, K, C, R)
% X: 3D N*3
% x: 2D N*2
% K: intrinsinc parameters
% C&R: camera pose
% output: blue 'x' --- reprojection points from 3D points
%         red 'o' ----- original 2D correspondences
%         error -------- average reprojection error 
img{1}=imread('./Milestone3_data/SfMProjectData_1/image0000001.bmp');
img{2}=imread('./Milestone3_data/SfMProjectData_1/image0000002.bmp');
img{3}=imread('./Milestone3_data/SfMProjectData_1/image0000003.bmp');
img{4}=imread('./Milestone3_data/SfMProjectData_1/image0000004.bmp');
img{5}=imread('./Milestone3_data/SfMProjectData_1/image0000005.bmp');
img{6}=imread('./Milestone3_data/SfMProjectData_1/image0000006.bmp');


n=size(x,1);
Xpos=[X ones(n,1)];

P=K*R*[eye(3) -C];
x_re0=P*Xpos';
x_re=x_re0(1:2,:)./x_re0(3,:);

figure(); imshow(img{imgnum}); hold on
plot(x_re(1,:),x_re(2,:),'cx',x(:,1),x(:,2),'ro');

error=sum(sqrt(sum((x-x_re').^2,2)))/n;
title(strcat('Camera# ',num2str(imgnum),' Average Reprojection Error: ',num2str(error)));

end


