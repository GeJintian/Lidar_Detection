function m = my_helperDrawCuboidFromROI(roi, ptCloud)
%helperDrawCuboidFromROI Helper function to disaply a cuboid from roi on
%   point cloud
%
% This is an example helper function that is subject to change or removal in
% future releases.

% Copyright 2020 The MathWorks, Inc.
subPc = select(ptCloud, findPointsInROI(ptCloud, roi));

zlim = subPc.ZLimits;
if isempty(zlim)
    zlim = ptCloud.ZLimits;
end
l = roi(2) - roi(1);
w = roi(4) - roi(3);
if isempty(zlim)
    h = roi(6) - roi(5);
    mz = (roi(6) + roi(5))/2;
else
    h = zlim(2) - zlim(1);
    mz = (zlim(1) + zlim(2))/2;
end

mx = (roi(1) + roi(2))/2;
my = (roi(4) + roi(3))/2;


m = cuboidModel([mx, my, mz, l, w, h, 0, 0, 0]);
end