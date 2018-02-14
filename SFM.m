init_script

a=3;
b=2; %a>b 
x1=frame.inliers{a-1,b}(:,1:2); %frame b column
x2=frame.inliers{a-1,b}(:,3:4); %frame a row

F = EstimateFundamentalMatrix(x1, x2);
E = EssentialMatrixFromFundamentalMatrix(F, K, K);
[Cset Rset] = ExtractCameraPose(E);

% Reject outlier correspondences. Use the first two images.
for i=1:4
    Xset{i} = LinearTriangulation(K, K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2);
end
[C, R, X0] = DisambiguateCameraPose(Cset, Rset, Xset); % Check the cheirality condition.

[X, x1re, x2re] = NonlinearTriangulation(K, K, zeros(3,1), eye(3), C, R, x1, x2, X0);

num=size(X,1);
traj=zeros(num,3+2*6);
traj(:,1:3)=X;
traj(:,(4+2*(b-1)):(4+2*(b-1)+1))=x1re;
traj(:,(4+2*(a-1)):(4+2*(a-1)+1))=x2re;

Csetnew=cell(1,6);
Rsetnew=cell(1,6);
Csetnew{b}=zeros(3,1); 
Csetnew{a}=C;
Rsetnew{b}=eye(3);
Rsetnew{a}=R;

error2=200;
iteration2=1000;

Xfinal=X;
num=2;
for i = 1 : 6 % Register camera and add 3D points for the rest of images
    counter=0;
    X3=[];
    x3=[];
    if i<b
        if size(frame.inliers{b-1,i},1)>size(frame.inliers{a-1,i},1)
            newmatch=frame.inliers{b-1,i};
            oldmatch=x1re;
            frameprev=b;
            xprev=newmatch(:,3:4);
            xcurr=newmatch(:,1:2);
        else
            newmatch=frame.inliers{a-1,i};
            oldmatch=x2re;
            frameprev=a;
            xprev=newmatch(:,3:4);
            xcurr=newmatch(:,1:2);
        end
        m=size(oldmatch,1);
        n=size(newmatch,1);
        for j=1:m
            for k=1:n
                if newmatch(k,3:4)==oldmatch(j,:)
                    counter=counter+1;
                    X3(counter,:)=X(j,:);
                    x3(counter,1:2)=newmatch(k,1:2);
                    x3(counter,3:4)=oldmatch(j,:);
                    traj(j,(4+2*(i-1)):(4+2*(i-1)+1))=x3(counter,1:2);
                end
            end
        end
    elseif i>a
       if size(frame.inliers{i-1,a},1)>size(frame.inliers{i-1,b},1)
            newmatch=frame.inliers{i-1,a};
            oldmatch=x2re;
            frameprev=a;
            xprev=newmatch(:,1:2);
            xcurr=newmatch(:,3:4);
        else
            newmatch=frame.inliers{i-1,b};
            oldmatch=x1re;
            frameprev=b;
            xprev=newmatch(:,1:2);
            xcurr=newmatch(:,3:4);
        end
        m=size(oldmatch,1);
        n=size(newmatch,1);
        for j=1:m
            for k=1:n
                if newmatch(k,1:2)==oldmatch(j,:)
                    counter=counter+1;
                    X3(counter,:)=X(j,:);
                    x3(counter,1:2)=newmatch(k,3:4);
                    x3(counter,3:4)=oldmatch(j,:);
                    traj(j,(4+2*(i-1)):(4+2*(i-1)+1))=x3(counter,1:2);
                end
            end
        end
%     elseif i<a && i>b
%         if size(frame.inliers{a-1,i},1)>size(frame.inliers{i-1,b},1)
%             newmatch=frame.inliers{a-1,i};
%             oldmatch=x2re;
%             t=1;
%             frameprev=a;
%             xprev=newmatch(:,3:4);
%             xcurr=newmatch(:,1:2);
%         else
%             newmatch=frame.inliers{i-1,b};
%             oldmatch=x1re;
%             t=0;
%             frameprev=b;
%             xprev=newmatch(:,1:2);
%             xcurr=newmatch(:,3:4);
%         end
%         m=size(oldmatch,1);
%         n=size(newmatch,1);
%         for j=1:m
%             for k=1:n
%                 if t==1
%                     if newmatch(k,3:4)==oldmatch(j,:)
%                         counter=counter+1;
%                        X3(counter,:)=X(j,:);
%                        x3(counter,1:2)=newmatch(k,1:2); 
%                        x3(counter,3:4)=oldmatch(j,:);
%                        traj(j,(4+2*(i-1)):(4+2*(i-1)+1))=x3(counter,1:2);                    
%                     end
%                 else
%                     if newmatch(k,1:2)==oldmatch(j,:)
%                         counter=counter+1;
%                        X3(counter,:)=X(j,:);
%                        x3(counter,1:2)=newmatch(k,3:4);
%                        x3(counter,3:4)=oldmatch(j,:);
%                        traj(j,(4+2*(i-1)):(4+2*(i-1)+1))=x3(counter,1:2);
%                     end
%                 end
%                 
%             end
%         end
    else
        continue;
    end

            
    C0=Csetnew{frameprev};
    R0=Rsetnew{frameprev};
    [Cnewli Rnewli] = PnPRANSAC(X3, x3(:,1:2), K, iteration2, error2); % Register the ith image.
    [Cnew Rnew] = NonlinearPnP(X3, x3(:,1:2), K, Cnewli, Rnewli);
    Csetnew{i}=Cnew;
    Rsetnew{i}=Rnew;
    Xnewli = LinearTriangulation(K, K, C0, R0, Cnew, Rnew, xprev, xcurr);
    [Xnew, xprevre, xcurrre] = NonlinearTriangulation(K, K, C0, R0, Cnew, Rnew, xprev, xcurr, Xnewli);% Add 3D
    Xfinal=[Xfinal;Xnew];
    trajnew=zeros(size(Xnew,1),3+2*6);
    trajnew(:,1:3)=Xnew;
    trajnew(:,(4+2*(frameprev-1)):(4+2*(frameprev-1)+1))=xprevre;
    trajnew(:,(4+2*(i-1)):(4+2*(i-1)+1))=xcurrre;
    traj=[traj;trajnew];
    V = BuildVisibilityMatrix(traj); % Get visibility matrix.points.
    num=num+1;
    
end

[Cba Rba Xba] = BundleAdjustment(num, Csetnew, Rsetnew, K, traj, V); 

