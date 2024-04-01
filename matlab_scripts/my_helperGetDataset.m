function reflidarData = my_helperGetDataset()
ptcloud = pcread('points.pcd');
point_xyz = ptcloud.Location;
point_real = zeros(size(point_xyz));
point_real(:,1)=point_xyz(:,3);
point_real(:,2)=point_xyz(:,2);
point_real(:,3)=point_xyz(:,1);
point_real = point_real(abs(point_real(:,2))<20,:);

reflidarData = pointCloud(point_real,Intensity=ptcloud.Intensity(find(abs(point_real(:,2))<20),:));

end