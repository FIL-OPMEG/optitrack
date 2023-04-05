function [sensorLevelRbTimeseries] = getChannelLevelRigidBodyTimeseries(cfg)
% This function outputs a cell array of tables containing rigid body
% information for each sensor position provided. The inputs are the
% sensorPositions from extractSensorPositions_V3, the rigidBodyT from
% readRigidBody and retroreflective marker placement information.
% Information about the rigid body definition is also required from Motive.
% 
% First, the rigid body markers are defined in Motive space and then in
% scannercast space. The transform between the two is found using permuted
% Kabsch method. That transform is applied to scannercast space sensor
% positions. The transformation matrix between the rigid body frame and
% individual sensor positions is calculated. These matrices are then
% applied to the whole rigidBodyT timeseries.
% 
% % Example use:
% % Read in sensor positions
% cfg				= [];
% cfg.folder        = 'C:\Users\nalexander\OneDrive - University College London\RobNicShared\Filbury OPM Study\stlfiles\sub-002Y\sensors';
% cfg.plot          = 'no';
% cfg.model			= 'C:\Users\nalexander\OneDrive - University College London\RobNicShared\Filbury OPM Study\stlfiles\sub-002Y\002Y.stl';
% cfg.outputfolder  = [];
% cfg.output        = {'G3','G2_shim'};
% cfg.flipX			= true;
% sensorPositions	= extractSensorPositions_V3(cfg);
% 
% % Read in the optitrack data
% cfg				= [];
% cfg.filename		= 'D:\data\tmp\sub-002Y_ses-002_task-retrieval_run-003.csv';
% rigidBodyT		= readRigidBody(cfg);
% 
% % Specify input for this function:
% cfg						= [];
% cfg.rigidBodyFile			= 'D:\data\tmp\002Y_scannercast.motive'; % Directory of rigid body definition exported from Motive. Must be the same as used in the recording
% cfg.sensorPositions		= sensorPositions;
% cfg.shortStalkSlots		= [54, 29, 19]; % specify the slots the stalks were on which length stalk was used.
% cfg.longStalkSlots		= [50, 40, 4];
% cfg.shortStalkTranslation	= [-1.65, -49.9, -5.35]; % These are default values which should not be changed unless you have changed the stalks. 
% cfg.longStalkTranslation	= [-1.65, -57.2, -5.35];
% cfg.rigidBodyT			= rigidBodyT.Scannercast_XXX.RigidBody; % Provide the rigid body timeseries table
% cfg.plot					= true; % Whether to plot some outputs. Recommended.
% [sensorLevelRbTimeseries] = getChannelLevelRigidBodyTimeseries(cfg);
% Author:	Nicholas Alexander (n.alexander@ucl.ac.uk)
% MIT License

%% Read the rigid body definition from the motive 
% Open the text file and read the contents
fid = fopen(cfg.rigidBodyFile, 'r');
fileContents = fread(fid, '*char').';
fclose(fid);

% Use a regular expression to find the position strings and extract the values
expression = '<position>([^<]+)</position>';
matches = regexp(fileContents, expression, 'tokens');
positions = cellfun(@(x) sscanf(x{1}, '%f,%f,%f'), matches,'UniformOutput',false);

% Reformat and scale
rigidPos = nan(length(positions),3);
for posIdx = 1:length(positions)
	rigidPos(posIdx,1) = positions{posIdx}(1,1)' * 1000;
	rigidPos(posIdx,2) = positions{posIdx}(2,1)' * 1000;
	rigidPos(posIdx,3) = positions{posIdx}(3,1)' * 1000;
end

% Normalise the positions
rigidCentroid = mean(rigidPos);
rigidPos = rigidPos - rigidCentroid;

clear matches expression positions fileContents fid posIdx ans

%% Find the marker positions in the same space as the scannercast
% Get positions of stalks from gen2 + shim with applied translation
shortStalkPos = nan(length(cfg.shortStalkSlots),3);
for i = 1:length(cfg.shortStalkSlots)
	% Find the sensor position row
	rowIdx = cfg.sensorPositions.slot == cfg.shortStalkSlots(i);

	% Get the orientation unit vectors
	tmpXOri = [cfg.sensorPositions.G2_Ox_X(rowIdx); cfg.sensorPositions.G2_Oy_X(rowIdx); cfg.sensorPositions.G2_Oz_X(rowIdx)];
	tmpYOri = [cfg.sensorPositions.G2_Ox_Y(rowIdx); cfg.sensorPositions.G2_Oy_Y(rowIdx); cfg.sensorPositions.G2_Oz_Y(rowIdx)];
	tmpZOri = [cfg.sensorPositions.G2_Ox_Z(rowIdx); cfg.sensorPositions.G2_Oy_Z(rowIdx); cfg.sensorPositions.G2_Oz_Z(rowIdx)];

	% Get the position of G2 with shim
	tmpPos = [cfg.sensorPositions.G2_P_X(rowIdx), cfg.sensorPositions.G2_P_Y(rowIdx), cfg.sensorPositions.G2_P_Z(rowIdx)];
	
	% Just move out a bit along Y axis
	shortStalkPos(i,:) = tmpPos + cfg.shortStalkTranslation(1)*tmpXOri' + cfg.shortStalkTranslation(2)*tmpYOri' + cfg.shortStalkTranslation(3)*tmpZOri';
end

longStalkPos = nan(length(cfg.longStalkSlots),3);
for i = 1:length(cfg.longStalkSlots)
	% Find the sensor position row
	rowIdx = cfg.sensorPositions.slot == cfg.longStalkSlots(i);

	% Get the orientation unit vectors
	tmpXOri = [cfg.sensorPositions.G2_Ox_X(rowIdx); cfg.sensorPositions.G2_Oy_X(rowIdx); cfg.sensorPositions.G2_Oz_X(rowIdx)];
	tmpYOri = [cfg.sensorPositions.G2_Ox_Y(rowIdx); cfg.sensorPositions.G2_Oy_Y(rowIdx); cfg.sensorPositions.G2_Oz_Y(rowIdx)];
	tmpZOri = [cfg.sensorPositions.G2_Ox_Z(rowIdx); cfg.sensorPositions.G2_Oy_Z(rowIdx); cfg.sensorPositions.G2_Oz_Z(rowIdx)];

	% Get the position of G2 with shim
	tmpPos = [cfg.sensorPositions.G2_P_X(rowIdx), cfg.sensorPositions.G2_P_Y(rowIdx), cfg.sensorPositions.G2_P_Z(rowIdx)];
	
	% Just move out a bit along Y axis
	longStalkPos(i,:) = tmpPos + cfg.longStalkTranslation(1)*tmpXOri' + cfg.longStalkTranslation(2)*tmpYOri' + cfg.longStalkTranslation(3)*tmpZOri';
end

fixedPos = [longStalkPos; shortStalkPos];
fixedCentroid = mean(fixedPos);
fixedPos = fixedPos - fixedCentroid;
clear short* long* tmp* i 

%% Now find the transform from fixedPos to rigidPos
% Create all permutations of the markers (if there are > 8 markers or so,
% this might need some more thought. So don't do that!)
permutations = perms(1:length(rigidPos(:,1)));

% Preallocate arrays to store results
errorDist = zeros(1,length(permutations(:,1)));

% Go through each combination, try and align them using Kabsch method,
% calculate error.
for i = 1:length(permutations(:,1))
    % Re-order fixedPos to pair with rigidPos (red to blue mapping)
    reorder = [1:length(rigidPos(:,1)); permutations(i,:)]';
    newFixedPos = fixedPos(reorder(:,2),:);
    
    % Calculate the cross-covariance matrix and perform SVD
    C = rigidPos' * newFixedPos;
    [U, ~, V] = svd(C);
    
    % Calculate the optimal rotation matrix
    R = V * U';
    
    % Rotate the set of vectors A using the rotation matrix R
    newFixedPos = newFixedPos * R;
    
    % Compute the distance between each pair of points
    distances = sqrt(sum((newFixedPos - rigidPos).^2, 2));
    
    % Compute the average distance between all pairs of points
    errorDist(i) = mean(distances);
end

% Warning if the error is a bit high.
if min(errorDist) > 5 % arbitrary 5mm
	warning("Error is a bit high. Make sure to check figures showing plot")
end


% Find the minimum error and apply that position order to fixedPos.
[~, minIdx] = min(errorDist);
reorder = [1:length(rigidPos(:,1)); permutations(minIdx,:)]';
fixedPos = fixedPos(reorder(:,2),:);

% Apply Kabsh again - saves storing all the R from permutations
% Calculate the cross-covariance matrix
C = rigidPos' * fixedPos;

% Perform the singular value decomposition of C
[U, ~, V] = svd(C);

% Calculate the optimal rotation matrix
R = V * U';

% Rotate the set of vectors A using the rotation matrix R
fixedPos = fixedPos * R;

% Plot the alignment
if cfg.plot
	figure
	hold on
	
	% plot blue points with a square marker
	scatter3(rigidPos(:,1), rigidPos(:,2), rigidPos(:,3), 20, 'b', 's', 'filled');
	
	% plot red points with a circle marker
	scatter3(fixedPos(:,1), fixedPos(:,2), fixedPos(:,3), 20, 'r', 'o', 'filled');
	
	% set plot properties
	xlabel('X')
	ylabel('Y')
	zlabel('Z')
	axis equal
	legend({'Rigid points', 'Aligned points'})
	
	hold off
end

%% Apply R to the sensor positions
% Get translation as difference in centroids
translation = rigidCentroid - fixedCentroid;

% Seems to work. Apply rotation to sensor positions and plot
transformedSensorPos2 = nan(height(cfg.sensorPositions),3);
transformedSensorPos3 = nan(height(cfg.sensorPositions),3);
transformedSensorOriX = nan(height(cfg.sensorPositions),3);
transformedSensorOriY = nan(height(cfg.sensorPositions),3);
transformedSensorOriZ = nan(height(cfg.sensorPositions),3);

for i = 1:height(cfg.sensorPositions)
	% Get positions
	tmpPos2 = [cfg.sensorPositions.G2_P_X(i), cfg.sensorPositions.G2_P_Y(i), cfg.sensorPositions.G2_P_Z(i)];
	tmpPos3 = [cfg.sensorPositions.G3_P_X(i), cfg.sensorPositions.G3_P_Y(i), cfg.sensorPositions.G3_P_Z(i)];
	
	% Get the orientation unit vectors
	tmpXOri = [cfg.sensorPositions.G2_Ox_X(i), cfg.sensorPositions.G2_Oy_X(i), cfg.sensorPositions.G2_Oz_X(i)];
	tmpYOri = [cfg.sensorPositions.G2_Ox_Y(i), cfg.sensorPositions.G2_Oy_Y(i), cfg.sensorPositions.G2_Oz_Y(i)];
	tmpZOri = [cfg.sensorPositions.G2_Ox_Z(i), cfg.sensorPositions.G2_Oy_Z(i), cfg.sensorPositions.G2_Oz_Z(i)];
	
	% Check the handedness is correct
	scalarTP = dot(tmpXOri,cross(tmpYOri,tmpZOri));

	% Flip if using x
	if scalarTP < 0
    	tmpXOri = -tmpXOri;
	end

	% Apply rotation - bit untidy...
	transformedSensorPos2(i,1:3) = (tmpPos2 + translation) * R;
	transformedSensorPos3(i,1:3) = (tmpPos3 + translation) * R;

	transformedSensorOriX(i,1:3) = tmpXOri * R;
	transformedSensorOriY(i,1:3) = tmpYOri * R;
	transformedSensorOriZ(i,1:3) = tmpZOri * R;
end

% Plot the transformed positions, along with the markers.
if cfg.plot
	figure;
	hold on;

	% Sensor frames (maybe different to channel orientations!)
	scatter3(transformedSensorPos2(:,1),transformedSensorPos2(:,2),transformedSensorPos2(:,3));
	for i = 1:height(cfg.sensorPositions)
		quiver3(transformedSensorPos2(i,1),transformedSensorPos2(i,2), transformedSensorPos2(i,3), transformedSensorOriX(i,1), transformedSensorOriX(i,2), transformedSensorOriX(i,3),10,'r');
		quiver3(transformedSensorPos2(i,1),transformedSensorPos2(i,2), transformedSensorPos2(i,3), transformedSensorOriY(i,1), transformedSensorOriY(i,2), transformedSensorOriY(i,3),10,'g');
		quiver3(transformedSensorPos2(i,1),transformedSensorPos2(i,2), transformedSensorPos2(i,3), transformedSensorOriZ(i,1), transformedSensorOriZ(i,2), transformedSensorOriZ(i,3),10,'b');
	end

	% Markers
	for i = 1:length(rigidPos)
		scatter3(rigidPos(i,1),rigidPos(i,2),rigidPos(i,3),10,'k');
	end
	
	% And an axis showing the rigid body
	rbPos = [0 0 0];
	rbXOri = [1 0 0];
	rbYOri = [0 1 0];
	rbZOri = [0 0 1];
	quiver3(rbPos(1),rbPos(2),rbPos(3),rbXOri(1),rbXOri(2),rbXOri(3),100,'r');
	quiver3(rbPos(1),rbPos(2),rbPos(3),rbYOri(1),rbYOri(2),rbYOri(3),100,'g');
	quiver3(rbPos(1),rbPos(2),rbPos(3),rbZOri(1),rbZOri(2),rbZOri(3),100,'b');
	
	hold off
end

clear tmp* new* U V translation 

%% Get 6DOF rigid body data for each sensor frame.
% Calculate the transformation matrix from the rigid body frame to each
% sensor frame
T = cell(length(transformedSensorPos2(:,1)),2);
for i = 1:length(transformedSensorPos2(:,1))
	% Get the orientation unit vectors
	tmpXOri = transformedSensorOriX(i,:);
	tmpYOri = transformedSensorOriY(i,:);
	tmpZOri = transformedSensorOriZ(i,:);

	% Compute the translation vector between the frames
	t1 = transformedSensorPos2(i,:) - rbPos;
	t2 = transformedSensorPos3(i,:) - rbPos;
	
	% Compute the rotation matrix between the frames
	R = [dot(rbXOri,tmpXOri) dot(rbXOri,tmpYOri) dot(rbXOri,tmpZOri);
		dot(rbYOri,tmpXOri) dot(rbYOri,tmpYOri) dot(rbYOri,tmpZOri);
		dot(rbZOri,tmpXOri) dot(rbZOri,tmpYOri) dot(rbZOri,tmpZOri)];


	% Combine the translation and rotation into a single transformation matrix
	T{i,1} = [R t1'; 0 0 0 1];
	T{i,2} = [R t2'; 0 0 0 1];
end

clear tmp* transformed*

%% Apply the transformation mat
% Breakdown the rigidbody timeseries
xPos = cfg.rigidBodyT.X_Position;
yPos = cfg.rigidBodyT.Y_Position;
zPos = cfg.rigidBodyT.Z_Position;
xAngle = cfg.rigidBodyT.X_Rotation;
yAngle = cfg.rigidBodyT.Y_Rotation;
zAngle = cfg.rigidBodyT.Z_Rotation;

% Preallocation
sensorLevelRbTimeseries = cell(height(cfg.sensorPositions),2);
for i = 1:numel(sensorLevelRbTimeseries)
    sensorLevelRbTimeseries{i} = zeros(height(cfg.rigidBodyT), 7);
end

% This takes a while, so update user
fprintf('Applying transformations to timeseries: 0%%\n');

% Loop through the transformation matrix array
for i = 1:height(cfg.sensorPositions)
	for j = 1:2
		
		% Convert Euler angles to rotation matrices
		R = eul2rotm([xAngle, yAngle, zAngle]);

		% Create 4x4 homogeneous transformation matrices for each time step
		M = zeros(4, 4, length(xPos));
		M(1:3, 1:3, :) = R;
		M(1, 4, :) = xPos;
		M(2, 4, :) = yPos;
		M(3, 4, :) = zPos;
		M(4, 4, :) = 1;

		% Apply the transformation matrix to each time step
		M_transformed = zeros(4, 4, length(xPos));
		for k = 1:length(xPos)
    		M_transformed(:,:,k) = T{i,j} * M(:,:,k);
		end

		% Extract the transformed position and orientation data
		transformedXPos = squeeze(M_transformed(1, 4, :));
		transformedYPos = squeeze(M_transformed(2, 4, :));
		transformedZPos = squeeze(M_transformed(3, 4, :));
		
% 		tmpEul = rotm2eul(M_transformed(1:3,1:3,:),'XYZ');
		tmpQuat = rotm2quat(M_transformed(1:3,1:3,:));
		sensorLevelRbTimeseries{i,j}(:,:) = [transformedXPos transformedYPos transformedZPos tmpQuat];
	end

	% Update user
	  % update progress
    if mod(i, round(height(cfg.sensorPositions)/100)) == 0 % update every 1%
        fprintf('\b\b\b\b%3d%%', round(i/height(cfg.sensorPositions)*100));
    end
end

% Format update
fprintf('\height(sensorPositions)');
fprintf('\n');

% Plot positition timeseries to show variety of sensor movement
if cfg.plot
	colormap = flipud(gray(length(sensorLevelRbTimeseries)));
	figure

	% X axis
	subplot(3,1,1)
	hold on
	col = 1;
	for i = 1:length(sensorLevelRbTimeseries)
		plot(sensorLevelRbTimeseries{i,1}(:,col) - mean(sensorLevelRbTimeseries{i,1}(:,col)), 'Color', colormap(i,:));
	end
	plot(xPos - mean(xPos), 'Color', 'r');
	ylabel('X Value')
	hold off
	
	% X axis
	subplot(3,1,2)
	hold on
	col = 2;
	for i = 1:length(sensorLevelRbTimeseries)
		plot(sensorLevelRbTimeseries{i,1}(:,col) - mean(sensorLevelRbTimeseries{i,1}(:,col)), 'Color', colormap(i,:));
	end
	plot(yPos - mean(yPos), 'Color', 'r');
	ylabel('Y Value')% X axis
	
	subplot(3,1,3)
	hold on
	col = 3;
	for i = 1:length(sensorLevelRbTimeseries)
		plot(sensorLevelRbTimeseries{i,1}(:,col) - mean(sensorLevelRbTimeseries{i,1}(:,col)), 'Color', colormap(i,:));
	end
	plot(zPos - mean(zPos), 'Color', 'r');
	ylabel('Z Value')

	hold off
end

%% Create output
% Table to match input
tmpFrame = 1:length(sensorLevelRbTimeseries{i,j});
tmpTime = tmpFrame / 120;
outputVars = {'Frame', 'Time', 'X_Position', 'Y_Position', 'Z_Position', 'X_Rotation', 'Y_Rotation', 'Z_Rotation', 'W_Rotation', 'MeanMarkerError'};
for i = 1:length(sensorLevelRbTimeseries(:,1))
	for j = 1:2
		
		tmpDat = [tmpFrame', tmpTime', sensorLevelRbTimeseries{i,j}, cfg.rigidBodyT.MeanMarkerError];		
		tmpTable = array2table(tmpDat,"VariableNames",outputVars);
		sensorLevelRbTimeseries{i,j} = tmpTable;
	end
end