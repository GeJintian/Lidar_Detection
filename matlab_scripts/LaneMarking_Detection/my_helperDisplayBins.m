function my_helperDisplayBins(x, y, delta, ptCloud, color)
%helperDisplayBins Helper function to display bins on a point cloud
%
% This is an example helper function that is subject to change or removal in
% future releases.

% Copyright 2020 The MathWorks, Inc.
zlim = ptCloud.ZLimits;
nBbox = numel(x);

hold on;
for i=1:nBbox-1
    roi = [x(i), x(i+1), y-delta, y+delta,  zlim];
     m = my_helperDrawCuboidFromROI(roi, ptCloud);
     ax = plot(m);
     ax.FaceColor = color;
     ax.FaceAlpha = 1;
end
hold off
end