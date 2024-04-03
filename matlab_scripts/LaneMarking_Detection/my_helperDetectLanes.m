function detectedLanePoints = my_helperDetectLanes(ptCloud, horizontalBinResolution,...
    numVerticalBins, startLanePoints, display)
%helperDetectLanes Helper function to detect and display lane on a point
%   cloud
%
% This is an example helper function that is subject to change or removal in
% future releases.

% Copyright 2020 The MathWorks, Inc.

if display
    figure;
    pcshow(ptCloud);
    title('Sliding Window Search')
    view(2);
    hold on;
end
numLanes = numel(startLanePoints);
verticalBins = zeros(numVerticalBins, 3, numLanes);
lanes = zeros(numVerticalBins, 3, numLanes);
laneStartX = linspace(ptCloud.XLimits(1), ptCloud.XLimits(2), numVerticalBins);
for i = 1:numVerticalBins-1
    for j = 1:numLanes
        laneStartY = startLanePoints(j); 
        % Define a vertical roi window
        roi = [laneStartX(i), laneStartX(i+1), laneStartY - horizontalBinResolution/2,...
            laneStartY + horizontalBinResolution/2, -inf, inf];
        tmpPc = select(ptCloud, findPointsInROI(ptCloud, roi));
        
        if ~isempty(tmpPc.Location)           
            [~, maxIndex] = max(tmpPc.Intensity);
            val = tmpPc.Location(maxIndex, :);
            verticalBins(i, :, j) = val;
            lanes(i, :, j) = val;
            % Slide the window with the update mean value along
            % y direction
            startLanePoints(j) = val(2);
            if display
                m = my_helperDrawCuboidFromROI(roi, ptCloud);
                ax = plot(m);
                ax.FaceColor = 'yellow';
                ax.FaceAlpha = 1;
            end
            % For dash lanes update the sliding window by 2D polynomial
        else
            value = lanes(2:end, 1:2, j);
            value(all(value == 0, 2), :)=[];
                        
            if size(value, 1) == 2 % Use linear prediction
                P = fitPolynomialRANSAC(value, 1, 0.1);
            elseif size(value, 1) > 2 % Use 2 degree polynomial prediction
                P = fitPolynomialRANSAC(value, 2, 0.1);
            else
                verticalBins(i, :, j) = verticalBins(end, :, j);
                continue;
            end
            error =  mean(sqrt((polyval(P, value(:, 1)) - value(:, 2)).^2));
            if error < 0.1
                xval = (roi(1) + roi(2))/2;
                yval = polyval(P, xval);
                
                % Use error to regularize the value of predicted y
                yval = yval - error*abs(yval);
                startLanePoints(j) = yval;
                roi(3:4) = [yval - horizontalBinResolution, ...
                    yval + horizontalBinResolution];
                % Update the lane point with the centre of the
                % predicted window
                tmpPc = select(ptCloud, findPointsInROI(ptCloud, roi));
                zmean = mean(tmpPc.Location(:, 3));
                verticalBins(i, :, j) = [xval, yval, zmean];
                if display
                    m = my_helperDrawCuboidFromROI(roi, ptCloud);
                    ax = plot(m);
                    ax.FaceColor = 'green';
                    ax.FaceAlpha = 1;
                end
            else
                roi(3:4) = [startLanePoints(j) - horizontalBinResolution, ...
                    startLanePoints(j) + horizontalBinResolution];
                if display
                    m = my_helperDrawCuboidFromROI(roi, ptCloud);
                    ax = plot(m);
                    ax.FaceColor = 'green';
                    ax.FaceAlpha = 1;
                end
            end
            
        end
    end
end
lane1 = lanes(:, :, 1);
lane2 = lanes(:, :, 2);
lane1(all(lane1 == 0, 2), :) = [];
lane2(all(lane2 == 0, 2), :) = [];
detectedLanePoints{1} = lane1;
detectedLanePoints{2} = lane2;
end