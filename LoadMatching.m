M1 = dlmread('./Milestone3_data/SfMProjectData_1/matching1.txt','',1, 0);
M2 = dlmread('./Milestone3_data/SfMProjectData_1/matching2.txt','',1, 0);
M3 = dlmread('./Milestone3_data/SfMProjectData_1/matching3.txt','',1, 0);
M4 = dlmread('./Milestone3_data/SfMProjectData_1/matching4.txt','',1, 0);
M5 = dlmread('./Milestone3_data/SfMProjectData_1/matching5.txt','',1, 0);

% RGB & u,v 
frame.points=cell(1,5);
frame.points{1,1}=M1(:,2:6);
frame.points{1,2}=M2(:,2:6);
frame.points{1,3}=M3(:,2:6);
frame.points{1,4}=M4(:,2:6);
frame.points{1,5}=M5(:,2:6);

frame.matching=cell(5,5);
for i=1:size(M1,1)
    n=M1(i,1);
    for j=1:(n-1)
        cframe=M1(i,7+3*(j-1));
        frame.matching{cframe-1,1}=[frame.matching{cframe-1,1};M1(i,5:6) M1(i,(7+3*(j-1)+1):(7+3*(j-1)+2))];
    end
end


for i=1:size(M2,1)
    n=M2(i,1);
    for j=1:(n-1)
        cframe=M2(i,7+3*(j-1));
        frame.matching{cframe-1,2}=[frame.matching{cframe-1,2};M2(i,5:6) M2(i,(7+3*(j-1)+1):(7+3*(j-1)+2))];
    end
end

for i=1:size(M3,1)
    n=M3(i,1);
    for j=1:(n-1)
        cframe=M3(i,7+3*(j-1));
        frame.matching{cframe-1,3}=[frame.matching{cframe-1,3};M3(i,5:6) M3(i,(7+3*(j-1)+1):(7+3*(j-1)+2))];
    end
end

for i=1:size(M4,1)
    n=M4(i,1);
    for j=1:(n-1)
        cframe=M4(i,7+3*(j-1));
        frame.matching{cframe-1,4}=[frame.matching{cframe-1,4};M4(i,5:6) M4(i,(7+3*(j-1)+1):(7+3*(j-1)+2))];
    end
end

for i=1:size(M5,1)
    n=M5(i,1);
    for j=1:(n-1)
        cframe=M5(i,7+3*(j-1));
        frame.matching{cframe-1,5}=[frame.matching{cframe-1,5};M5(i,5:6) M5(i,(7+3*(j-1)+1):(7+3*(j-1)+2))];
    end
end

