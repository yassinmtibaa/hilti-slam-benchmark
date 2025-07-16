function scans = readHiltiBag(bagPath, topic)
/% READHILTIBAG  Convert ROS 1/2 bag to organised pointCloud cells.
%   scans = readHiltiBag("data/site3_handheld_1.bag","/hesai/pandar")
%/

if nargin < 2 || isempty(topic)
    topic = "/hesai/pandar";
end

[~,~,ext] = fileparts(bagPath);
if ext==".bag"
    bag = rosbagreader(bagPath,"FastStart",true);   % ROS 1
else
    bag = ros2bagreader(bagPath,"FastStart",true);  % ROS 2
end

sel  = select(bag,"Topic",topic);
msgs = readMessages(sel,"DataFormat","struct");
assert(~isempty(msgs),"Topic not found or empty bag");

rings = 40;                               % Pandar40 (try 64 if needed)
elev  = linspace(-16,15,rings);           % simple evenly-spaced model
lp    = lidarParameters(rings,360,'ElevationAngles',elev);

n = numel(msgs);  scans = cell(1,n);
parfor k = 1:n
    xyz      = rosReadXYZ(msgs{k});
    scans{k} = pcorganize(pointCloud(xyz),lp);
end
end
