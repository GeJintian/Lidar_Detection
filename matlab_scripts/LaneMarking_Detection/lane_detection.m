ptCloud = my_helperGetDataset;

% Define ROI in meters
xlim = [2 100];
ylim = [-20 15];
zlim = [-5 1];
roi = [xlim ylim zlim];

% Crop point cloud using ROI
indices = findPointsInROI(ptCloud,roi);
croppedPtCloud = select(ptCloud,indices);

% Remove ground plane
maxDistance = 0.1;
referenceVector = [0 0 1];
[model,inliers,outliers] = pcfitplane(croppedPtCloud,maxDistance,referenceVector);
groundPts = select(croppedPtCloud,inliers);


%%%%%% Lane Point Detection
histBinResolution = 0.2;
[histVal,yvals] = my_helperComputeHistogram(groundPts,histBinResolution);

figure
plot(yvals,histVal,'--k')
set(gca,'XDir','reverse')
hold on

[peaks,locs] = my_helperfindpeaks(histVal);
startYs = yvals(locs);

laneWidth = 4;
[startLanePoints,detectedPeaks] = my_helperInitialWindow(startYs,peaks,laneWidth);

plot(startYs,peaks,'*r')
plot(startLanePoints,detectedPeaks,'og')
legend('Histogram','Peak','Detected Peaks','Location','North')
title('Peak Detection')
hold off


%%%%%% Window Search
vBinRes = 1;
hBinRes = 0.8;
numVerticalBins = ceil((groundPts.XLimits(2) - groundPts.XLimits(1))/vBinRes);
laneStartX = linspace(groundPts.XLimits(1),groundPts.XLimits(2),numVerticalBins);

% Display bin windows
figure
pcshow(groundPts)
view(2)
my_helperDisplayBins(laneStartX,startLanePoints(1),hBinRes/2,groundPts,'red');
my_helperDisplayBins(laneStartX,startLanePoints(2),hBinRes/2,groundPts,'blue');
title('Initialized Sliding Windows')

display = true;
lanes = my_helperDetectLanes(groundPts,hBinRes, ...
    numVerticalBins,startLanePoints,display);

% Plot final lane points
lane1 = lanes{1};
lane2 = lanes{2};

figure
pcshow(groundPts)
title('Detected Lane Points')
hold on
p1 = plot3(lane1(:,1),lane1(:,2),lane1(:,3),'*y');
p2 = plot3(lane2(:,1),lane2(:,2),lane2(:,3),'*r');
hold off
view(2)
lgnd = legend([p1 p2],{'Left Lane Points','Right Lane Points'});
set(lgnd,'color','White','Location','southoutside')


%%%%%% Lane Fitting
[P1,error1] = my_helperFitPolynomial(lane1(:,1:2),2,0.1);
[P2,error2] = my_helperFitPolynomial(lane2(:,1:2),2,0.1);

xval = linspace(5,40,80);
yval1 = polyval(P1,xval);
yval2 = polyval(P2,xval);

% Z-coordinate estimation
modelParams = model.Parameters;
zWorld1 = (-modelParams(1)*xval - modelParams(2)*yval1 - modelParams(4))/modelParams(3);
zWorld2 = (-modelParams(1)*xval - modelParams(2)*yval2 - modelParams(4))/modelParams(3);

% Visualize fitted lane
figure
pcshow(croppedPtCloud)
title('Fitted Lane Polynomial')
hold on
p1 = plot3(xval,yval1,zWorld1,'y','LineWidth',0.2);
p2 = plot3(xval,yval2,zWorld2,'r','LineWidth',0.2);
lgnd = legend([p1 p2],{'Left Lane Points','Right Lane Points'});
set(lgnd,'color','White','Location','southoutside')
view(2)
hold off

lane3d1 = [xval' yval1' zWorld1'];
lane3d2 = [xval' yval2' zWorld2'];

% Shift the polynomial with a high score along the Y-axis towards
% the polynomial with a low score
if error1 > error2
    lanePolynomial = P2;
    if lane3d1(1,2) > 0
        lanePolynomial(3) = lane3d2(1,2) + laneWidth;
    else
        lanePolynomial(3) = lane3d2(1,2) - laneWidth;
    end
    lane3d1(:,2) = polyval(lanePolynomial,lane3d1(:,1));
    lanePolynomials = [lanePolynomial; P2];
else
    lanePolynomial = P1;
    if lane3d2(1,2) > 0
        lanePolynomial(3) = lane3d1(1,2) + laneWidth;
    else
        lanePolynomial(3) = lane3d1(1,2) - laneWidth;
    end
    lane3d2(:,2) = polyval(lanePolynomial,lane3d2(:,1));
    lanePolynomials = [P1; lanePolynomial]
end

% Visualize lanes after parallel fitting
figure
pcshow(ptCloud,ColorSource = "Intensity")
axis([0 50 -15 15 -5 5])
hold on
p1 = plot3(lane3d1(:,1),lane3d1(:,2),lane3d1(:,3),'y','LineWidth',0.2);
p2 = plot3(lane3d2(:,1),lane3d2(:,2),lane3d2(:,3),'r','LineWidth',0.2);
view([-90 90])
title('Fitted Lanes')
lgnd = legend([p1 p2],{'Left Lane Points','Right Lane Points'});
set(lgnd,'color','White','Location','southoutside')
hold off


%%%%%% Lane Tracking
% Initial values
curveInitialParameters = lanePolynomials(1,1:2);
driftInitialParameters = lanePolynomials(:,3)';
initialEstimateError = [1 1 1]*1e-1;
motionNoise = [1 1 1]*1e-7;
measurementNoise = 10;

% Configure Kalman filter
curveFilter = configureKalmanFilter('ConstantAcceleration', ...
    curveInitialParameters,initialEstimateError,motionNoise,measurementNoise);
driftFilter = configureKalmanFilter('ConstantAcceleration', ...
    driftInitialParameters,initialEstimateError,motionNoise,measurementNoise);