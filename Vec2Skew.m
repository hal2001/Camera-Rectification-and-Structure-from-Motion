function skew=Vec2Skew(v)
% turn a vector into its skew-symmetric matrix

skew=[0 -v(3) v(2);
    v(3) 0 -v(1);
    -v(2) v(1) 0];
   
end