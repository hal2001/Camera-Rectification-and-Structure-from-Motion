clear all
clc
LoadMatching

K = [568.996140852 0 643.21055941;
     0 568.988362396 477.982801038;
     0 0 1];

frame.inliers=cell(5,5);
iteration=1000;
error=0.01;       
            
for i=1:5
    for j=1:5
        if ~isempty(frame.matching{i,j})
            [frame.inliers{i,j}(:,1:2) frame.inliers{i,j}(:,3:4)] = GetInliersRANSAC(...
                frame.matching{i,j}(:,1:2), frame.matching{i,j}(:,3:4), error, iteration);
        end
    end
end

img{1}=imread('./Milestone3_data/SfMProjectData_1/image0000001.bmp');
img{2}=imread('./Milestone3_data/SfMProjectData_1/image0000002.bmp');
img{3}=imread('./Milestone3_data/SfMProjectData_1/image0000003.bmp');
img{4}=imread('./Milestone3_data/SfMProjectData_1/image0000004.bmp');
img{5}=imread('./Milestone3_data/SfMProjectData_1/image0000005.bmp');
img{6}=imread('./Milestone3_data/SfMProjectData_1/image0000006.bmp');


