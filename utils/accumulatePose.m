function row = accumulatePose(prev, T)
/% prev = [t x y z qw qx qy qz]  (TUM order) %/
R   = T.R;
t   = T.Translation.';
qT  = rotm2quat(R);          % qw qx qy qz
q0  = prev(5:8);
xyz = prev(2:4).' + quat2rotm(q0) * t;
q   = quatmultiply(q0, qT);
row = [0 xyz.' q];
end
