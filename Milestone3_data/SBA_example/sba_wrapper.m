function [cP, X] = sba_wrapper(measurements, cP, X, K)

% cP: camera poses
% X: 3D points
% measurements: 2D points
% K: intrinsic parameter

nFrames = length(cP);
nFeatures = length(X);

r0 = [];    cams = [];  pts2D = [];
spmask=sparse([], [], [], 1, nFrames);

% Concaternate all camera poses into cams vector
for i = 1 : nFrames   
    q = Matrix2Quaternion(inv(K)*cP{i}(1:3,1:3));
    q = QuaternionNormalization(q);
    r0 = [r0;q'];
    cams = [cams;q' GetCameraPosition(cP{i}, K)'];
end

% Concaternate all 2D measurements into pts2D
for i = 1 : nFeatures
    pts2 = [];
    for j = 1 : nFrames
        if measurements(2*(i-1)+1,j) == 0
            continue;
        end
        spmask(i,j) = 1;
        pts2 = [pts2 measurements(2*(i-1)+1:2*i,j)'];
    end
    pts2D = [pts2D pts2];
end

pts3D = X;

% ignore this
r0=reshape(r0', 1, numel(r0));

% Calibration parameters
cal = [K(1,1) K(1,2), K(1,3), K(2,2), K(2,3)];

opts=[1E-1, 0, 0, 1E-5, 0.0];
p0=[Mat2Vec(cams)' Mat2Vec(pts3D)'];

if isreal(p0) ~= 1
    k = 1;
end
[ret, p, info]=sba(nFeatures, 0, nFrames, 1, spmask, p0, 7, 3, pts2D, 2, 'projection', 1e+2, 1, opts, 'motstr', r0, cal);

% Retrieve paramters
for i = 1 : nFrames
    camera = p(7*(i-1)+1:7*i)';
    R = Quaternion2Matrix(camera(1:4));
    C = camera(5:end);
    cP{i} = K*R*[eye(3), -C];
end

X = [];
for i = 1 : nFeatures
    pts3 = p(7*nFrames+3*(i-1)+1:7*nFrames+3*i);
    X = [X;pts3];
end
