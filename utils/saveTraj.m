function saveTraj(fname, traj)
/% traj = N×8  [t x y z qw qx qy qz] %/
[fpath,~,~] = fileparts(fname);
if ~isfolder(fpath), mkdir(fpath); end
writematrix(traj,fname,"Delimiter"," ");
end
