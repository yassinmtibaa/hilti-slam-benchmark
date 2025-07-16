function traj = pipeline(bagFile, topic)
/% Continuous LOAM odometry for a full bag. %/

if nargin<2, topic="/hesai/pandar"; end
scans = readHiltiBag(bagFile, topic);
N     = numel(scans);

traj        = zeros(N,8);
traj(1,5:8) = [1 0 0 0];                % identity quaternion
Tprev       = rigidtform3d;

for k = 2:N
    T = pcregisterloam(scans{k}, scans{k-1}, ...
            "InitialTransform",Tprev,"GridStep",0.4);
    traj(k,:) = accumulatePose(traj(k-1,:), T);
    Tprev = T;
    if mod(k,300)==0
        fprintf("Frame %d / %d (%.1f%%)\n",k,N,100*k/N);
    end
end

traj(:,1) = (0:N-1).' * 0.1;            % assume 10 Hz

outFile = fullfile("results", ...
          replace(bagFile,["\",".bag",".db3"],["_traj_loam",".tum",".tum"]));
saveTraj(outFile,traj);
fprintf("LOAM finished → %s\n",outFile);
end
