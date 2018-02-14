function E = EssentialMatrixFromFundamentalMatrix(F, K1, K2)
 % (INPUT) F: 3 * 3 fundamental matrix 
 % (INPUT) K1 and K2: 3 * 3 calibration matrices 
 % (OUTPUT) E: 3 * 3 essential matrix.

% E = K1'*F*K2;
E = K2'*F*K1;
[u, d, v] = svd(E);
d(1,1) = 1; 
d(2,2) = 1; 
d(3,3) = 0;
E = u*d*v';

end