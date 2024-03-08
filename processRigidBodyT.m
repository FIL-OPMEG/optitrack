function [pos, euler, rigidBodySpeed, markerSpeedAvg, angularVelocityMag] = processRigidBodyT(cfg,rigidBodyT)
% Function to plot rigid body positions.
%
% Example use:
% % Get exported mocap data filename
% % Setup bids
% bids = [];
% bids.sub = '001';
% bids.ses = '001';
% bids.run = '001';
% bids.task = 'yourTask';
% bids.directory = 'D:/YOUR/BIDS/FOLDER';
% 
% % BIDS spec cfg
% cfg = [];
% cfg.category	= 'mot'; 
% cfg.description	= 'ScannercastRigidBody'; 
% cfg.type		= '.csv'; 
% cfg.derivative	= false;
% cfg.detailed	= true;
% [~, ~, combined] = bidsFilename(cfg, bids);
% 
% % Read in the optitrack data
% cfg			= [];
% cfg.filename	= combined;
% rigidBodyT	= readRigidBody(cfg);
% 
% % Plot
% cfg					= [];
% cfg.rigidBodyLabel	= 'Scannercast';
% cfg.lowpass			= 5;
% cfg.plot				= true;
% cfg.timeWindow		= 30;
% cfg.rotSmooth			= 0.3;
% cfg.unwrapRot			= true;
% cfg.sgOrder			= 2;
% cfg.sgFreq			= 3;
% [pos, euler, rigidBodySpeed, markerSpeedAvg, angularVelocityMag] = processRigidBodyT(cfg,rigidBodyT);


%% Start of function
% Filter design for positions
fs = round(str2double(rigidBodyT.cfg.ExportFrameRate));
if (isfield(cfg,'lowpass') && ~isempty(cfg.lowpass))
	d = designfilt('lowpassiir', 'FilterOrder', 8, 'HalfPowerFrequency',cfg.lowpass,...
		'SampleRate', fs);
else
	d = [];
end

%% Process rigid body marker data
% Identify number of markers on rigid body.
cols = fieldnames(rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker);
posMarkerCount = zeros(1,length(cols));
for fieldIdx = 1:length(cols)
	if regexp(cols{fieldIdx},'Marker\d.*')
		posMarkerCount(fieldIdx) = str2double(regexp(cols{fieldIdx},'\d+','match'));
	end
end
numMarkers = max(posMarkerCount);

% Get the trajectories (dim = xyz_marker_t)
markerSpeed = zeros(length(rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker.('Marker1_X_Position')) - 1,numMarkers);
traj = zeros(3,numMarkers,length(rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker.('Marker1_X_Position')));
qual = zeros(numMarkers,length(rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker.('Marker1_X_Position')));
for markIdx = 1:numMarkers
	traj(1,markIdx,:) = rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker.(['Marker',num2str(markIdx),'_X_Position']);
	traj(2,markIdx,:) = rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker.(['Marker',num2str(markIdx),'_Y_Position']);
	traj(3,markIdx,:) = rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker.(['Marker',num2str(markIdx),'_Z_Position']);

	% Lowpass filter
	if ~isempty(d)
		traj(1,markIdx,:) = filtfilt(d,squeeze(traj(1,markIdx,:)));
		traj(2,markIdx,:) = filtfilt(d,squeeze(traj(2,markIdx,:)));
		traj(3,markIdx,:) = filtfilt(d,squeeze(traj(3,markIdx,:)));
	end

	% When quality is 0 (i.e. no observed data) set traj to NaN
	qual(markIdx,:) = rigidBodyT.(cfg.rigidBodyLabel).RigidBodyMarker.(['Marker',num2str(markIdx),'_MarkerQuality']);
	traj(:,markIdx,~(qual(markIdx,:) > 0)) = NaN;

	% Get adjacent trajectories
	dt = 1/round(str2double(rigidBodyT.cfg.ExportFrameRate));
	
	tmpX = squeeze(traj(1,markIdx,:));
	tmpY = squeeze(traj(2,markIdx,:));
	tmpZ = squeeze(traj(3,markIdx,:));
	
	% Create 3D arrays for the current and previous positions
	prevPos = cat(2, tmpX(1:end-1), tmpY(1:end-1), tmpZ(1:end-1));
	currPos = cat(2, tmpX(2:end), tmpY(2:end), tmpZ(2:end));
	
	% Compute the linear velocities
	linearVelocity = (currPos - prevPos) / dt;
	
	% Compute the magnitudes of the linear velocities (speed)
	markerSpeed(:,markIdx) = sqrt(sum(linearVelocity.^2, 2));
	
end

% Process the marker velocity data
% NaN mean
markerSpeedAvg = arrayfun(@(x) mean(markerSpeed(x,~isnan(markerSpeed(x,:)))), 1:size(markerSpeed,1));
markerNaNs = isnan(markerSpeedAvg);

iVec = 1:numel(markerSpeedAvg); 
markerSpeedAvg = interp1(iVec(~markerNaNs), markerSpeedAvg(~markerNaNs), iVec(~markerNaNs));

% Interpolate nans for s-g filter

if rem(fs,2) == 0
	framelen = (cfg.sgFreq * fs) + 1; 
else
	framelen = cfg.sgFreq * fs; 
end

markerSpeedAvg = sgolayfilt(markerSpeedAvg, cfg.sgOrder, framelen);

% Can be an issue with negative values (impossible). Set to zero.
zerosIdx = markerSpeedAvg < 0;
markerSpeedAvg(zerosIdx) = 0;
markerSpeedAvg(markerNaNs) = NaN;

%% Get position info of rigidbody (motive records Z = forward; Y = up, X = left)
% Make sure there are 3 or more good quality markers available.
% goodSegments = sum(qual > 0.8,1) > 2;
goodSegments = sum(rigidBodyT.Scannercast.RigidBody.MeanMarkerError > 0) > 2;

pos = [];
pos.X = rigidBodyT.(cfg.rigidBodyLabel).RigidBody.X_Position;
pos.Y = rigidBodyT.(cfg.rigidBodyLabel).RigidBody.Y_Position;
pos.Z = rigidBodyT.(cfg.rigidBodyLabel).RigidBody.Z_Position;

% Correct origin and filter for each channel
fields = fieldnames(pos);
for i = 1:numel(fields)
	% Get position info
	pos.(fields{i}) = rigidBodyT.(cfg.rigidBodyLabel).RigidBody.([fields{i},'_Position']);

	% Correct origin based on median
    pos.(fields{i}) = pos.(fields{i}) - median(pos.(fields{i}));

	% Lowpass filter
	if ~isempty(d)
		pos.(fields{i}) = filtfilt(d, pos.(fields{i}));
	end
	
	pos.(fields{i})(~goodSegments) = NaN;
end

%% Calculate the linear velocity and speed
% Time delta
dt = 1/round(str2double(rigidBodyT.cfg.ExportFrameRate));

% Offset arrays
prevPos = cat(2, pos.X(1:end-1), pos.Y(1:end-1), pos.Z(1:end-1));
currPos = cat(2, pos.X(2:end), pos.Y(2:end), pos.Z(2:end));

% Remove bad segments if they are in either current or previous pos
currPos(isnan(prevPos)) = NaN;
prevPos(isnan(currPos)) = NaN;

% Compute the linear velocity over time
linearVelocity = (currPos - prevPos) / dt;


% Compute the magnitudes of the linear velocities (speed mm/s)
rigidBodySpeed = sqrt(sum(linearVelocity.^2, 2));

rbNaNs = isnan(rigidBodySpeed);

iVec = 1:numel(rigidBodySpeed); 
rigidBodySpeed = interp1(iVec(~rbNaNs), rigidBodySpeed(~rbNaNs), iVec(~rbNaNs));


rigidBodySpeed = sgolayfilt(rigidBodySpeed, cfg.sgOrder, framelen);

% Can be an issue with negative values due to s-g filter. Impossible, so set to zero.
zerosIdx = rigidBodySpeed < 0;
rigidBodySpeed(zerosIdx) = 0;

rigidBodySpeed(rbNaNs) = NaN;

%% Get rotation info and apply SLERP low pass filter
quatRot = [];
fields = {'X','Y','Z','W'};
for i = 1:numel(fields)
    % Get rotation info
    quatRot.(fields{i}) = rigidBodyT.(cfg.rigidBodyLabel).RigidBody.([fields{i},'_Rotation']);

	% Apply SLERP low pass filter
	if (isfield(cfg,'rotSmooth') && ~isempty(cfg.rotSmooth))
    	for j = 2:length(quatRot.(fields{i}))
        	quatRot.(fields{i})(j) = slerp(quatRot.(fields{i})(j-1), quatRot.(fields{i})(j), cfg.rotSmooth);
    	end
	end
end

%% Convert quaternion to Euler angles
tmp = quat2eul([quatRot.W, quatRot.X, quatRot.Y, quatRot.Z]);

% Again, correcting for motive mixing up Y and Z...
euler.roll = tmp(:,1) * (180/pi);
euler.pitch = tmp(:,2) * (180/pi);
euler.yaw = tmp(:,3) * (180/pi);

eulerUnwrap.roll = unwrap(tmp(:,1),[],1) * (180/pi);
eulerUnwrap.pitch = unwrap(tmp(:,2),[],1) * (180/pi);
eulerUnwrap.yaw = unwrap(tmp(:,3),[],1) * (180/pi);

% Subtract the mean from each Euler angle to center them at zero
eulerUnwrap.roll = eulerUnwrap.roll - median(eulerUnwrap.roll);
eulerUnwrap.pitch = eulerUnwrap.pitch - median(eulerUnwrap.pitch);
eulerUnwrap.yaw = eulerUnwrap.yaw - median(eulerUnwrap.yaw);

%% Calculate the angular velocity
% Calculate derivatives
wx = gradient(deg2rad(eulerUnwrap.roll), dt)./dt; % x angular velocity in rad/s
wy = gradient(deg2rad(eulerUnwrap.pitch), dt)./dt; % y angular velocity in rad/s
wz = gradient(deg2rad(eulerUnwrap.roll), dt)./dt; % z angular velocity in rad/s

angularVelocity = [wx, wy, wz];

angularVelocityMag = zeros(length(eulerUnwrap.roll),1);
for i = 1:length(angularVelocity) - 1
	angularVelocityMag(i) = norm(angularVelocity(i+1) - angularVelocity(i));
end


%% Plot
if isfield(cfg,'plot') && cfg.plot
	% Number of samples to plot at a time
	samplesToPlot = cfg.timeWindow * round(str2double(rigidBodyT.cfg.ExportFrameRate));
	
	% Time axis, starting at 0
	t = (1:length(pos.X)) / round(str2double(rigidBodyT.cfg.ExportFrameRate))...
		- 1/round(str2double(rigidBodyT.cfg.ExportFrameRate));
	
	% Create a new figure
	figure;
	

	%% Create a subplot for the position plot
	subplot(4,3,1:2)
	
	% Initial plot
	plotHandleX = plot(t(1:samplesToPlot), pos.X(1:samplesToPlot), 'Color', [1 0.2 0.2], 'LineWidth', 2); hold on;
	plotHandleY = plot(t(1:samplesToPlot), pos.Y(1:samplesToPlot),  'Color', [0.2 1 0.2], 'LineWidth', 2);
	plotHandleZ = plot(t(1:samplesToPlot), pos.Z(1:samplesToPlot), 'Color', [0.2 0.2 1], 'LineWidth', 2); hold off;
	
	xlabel("Time (s)")
	ylabel("Position (mm)")

	%% Create a subplot for the Euler plot
	subplot(4,3,4:5)
	
	% Initial plot for Euler angles
	plotHandleRoll = plot(t(1:samplesToPlot), eulerUnwrap.roll(1:samplesToPlot), 'Color', [1 0.2 0.2], 'LineWidth', 2); hold on;
	plotHandlePitch = plot(t(1:samplesToPlot), eulerUnwrap.pitch(1:samplesToPlot), 'Color', [0.2 1 0.2], 'LineWidth', 2);
	plotHandleYaw = plot(t(1:samplesToPlot), eulerUnwrap.yaw(1:samplesToPlot), 'Color', [0.2 0.2 1], 'LineWidth', 2); hold off;
	
	xlabel("Time (s)")
	ylabel("Euler Rotation (deg)")

	%% Create a subplot for the 3D plot
	h1 = subplot(4,3,3);
	
	midpoint = round(mean([1, samplesToPlot]));

	% Define the origin
	origin = [pos.X(midpoint); pos.Y(midpoint); pos.Z(midpoint)];
	
	% Convert quaternion to rotation matrix
	% Assuming quaternion is in the form q = [w x y z]
	
	q = [quatRot.W(midpoint), quatRot.X(midpoint), quatRot.Y(midpoint), quatRot.Z(midpoint)];
	q = q/norm(q); % normalize quaternion
	w = q(1); 
	x = q(2); 
	y = q(3); 
	z = q(4);
	R = [1-2*y^2-2*z^2, 2*x*y-2*z*w, 2*x*z+2*y*w;
     	2*x*y+2*z*w, 1-2*x^2-2*z^2, 2*y*z-2*x*w;
     	2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x^2-2*y^2];
	
	% Rotate axes
	rot_axes = R * (eye(3) * 10);
	
	% Plot axes
	hold on;
	quiverHandleX = quiver3(origin(1), origin(2), origin(3), rot_axes(1,1), rot_axes(2,1), rot_axes(3,1), 'r');
	quiverHandleY = quiver3(origin(1), origin(2), origin(3), rot_axes(1,2), rot_axes(2,2), rot_axes(3,2), 'g');
	quiverHandleZ = quiver3(origin(1), origin(2), origin(3), rot_axes(1,3), rot_axes(2,3), rot_axes(3,3), 'b');
	hold off;

	axis equal;
	grid on;

	xlim([min(pos.X) max(pos.X)])
	ylim([min(pos.Y) max(pos.Y)])
	zlim([min(pos.Z) max(pos.Z)])
	view(h1, 0, 0);  

	% Copy to another subplot and change view
	% Copy the plot to another subplot
	h2 = subplot(4,3,6);
	copyobj(allchild(h1), h2);
	
	axis equal;
	grid on;

	xlim([min(pos.X) max(pos.X)])
	ylim([min(pos.Y) max(pos.Y)])
	zlim([min(pos.Z) max(pos.Z)])

	% Change the view in the new subplot
	% Replace azimuth and elevation with your desired view angles
	view(h2, 90, 90);  

	% And again
	h3 = subplot(4,3,9);
	copyobj(allchild(h1), h3);
	axis equal;
	grid on;
	xlim([min(pos.X) max(pos.X)])
	ylim([min(pos.Y) max(pos.Y)])
	zlim([min(pos.Z) max(pos.Z)])
	view(h3, 0, 90);  


	%% Plot rigid body speed
	subplot(4,3,7:8)
	plotHandleRbSpeed = plot(t(1:samplesToPlot), rigidBodySpeed(1:samplesToPlot), 'Color', 'k', 'LineWidth', 2); hold on;
	ylim([0, max(rigidBodySpeed)])

	xlabel("Time (s)")
	ylabel("Rigid Body Speed (mm/s)")

	%% Plot average marker speed 
	subplot(4,3,10:11)
	plotHandleMarkSpeed = plot(t(1:samplesToPlot), markerSpeedAvg(1:samplesToPlot), 'Color', 'k', 'LineWidth', 2); hold on;
	ylim([0, max(markerSpeedAvg)])

	xlabel("Time (s)")
	ylabel("Marker Avg Speed (mm/s)")


	%% Plot with slider to change time window shown.
	% Create a slider
	slider = uicontrol('Style', 'slider',...
    	'Min', 1, 'Max', length(pos.X) - samplesToPlot + 1,...
    	'Value', 1, 'Position', [150 20 300 20]);

	% Create a listener for the slider
	addlistener(slider, 'Value', 'PostSet', @(src, event) updatePlot(slider, plotHandleX, plotHandleY, plotHandleZ, ...
		plotHandleRoll, plotHandlePitch, plotHandleYaw, quiverHandleX, quiverHandleY, quiverHandleZ, plotHandleRbSpeed,...
		samplesToPlot, pos, eulerUnwrap, t, quatRot,rigidBodySpeed,h2,h3,plotHandleMarkSpeed,markerSpeedAvg));
	
	% Initial update
	updatePlot(slider, plotHandleX, plotHandleY, plotHandleZ, ...
		plotHandleRoll, plotHandlePitch, plotHandleYaw, quiverHandleX, quiverHandleY, quiverHandleZ, plotHandleRbSpeed,...
		samplesToPlot, pos, eulerUnwrap, t, quatRot,rigidBodySpeed,h2,h3,plotHandleMarkSpeed,markerSpeedAvg)
end

if (isfield(cfg,'unwrapRot') && cfg.unwrapRot)
	euler = eulerUnwrap;
end


end

function updatePlot(slider, plotHandleX, plotHandleY, plotHandleZ, ...
		plotHandleRoll, plotHandlePitch, plotHandleYaw, quiverHandleX, quiverHandleY, quiverHandleZ, plotHandleVel,...
		samplesToPlot, pos, eulerUnwrap, t, quatRot,rigidBodySpeed,h2,h3,plotHandleMarkSpeed,markerSpeedAvg)

    startIdx = round(slider.Value);
    endIdx = startIdx + samplesToPlot - 1;
    
    % Update plot data for position
    set(plotHandleX, 'XData', t(startIdx:endIdx), 'YData', pos.X(startIdx:endIdx));
    set(plotHandleY, 'XData', t(startIdx:endIdx), 'YData', pos.Y(startIdx:endIdx));
    set(plotHandleZ, 'XData', t(startIdx:endIdx), 'YData', pos.Z(startIdx:endIdx));
    
    % Update plot data for Euler angles
    set(plotHandleRoll, 'XData', t(startIdx:endIdx), 'YData', eulerUnwrap.roll(startIdx:endIdx));
    set(plotHandlePitch, 'XData', t(startIdx:endIdx), 'YData', eulerUnwrap.pitch(startIdx:endIdx));
    set(plotHandleYaw, 'XData', t(startIdx:endIdx), 'YData', eulerUnwrap.yaw(startIdx:endIdx));
    
    % Update 3D plot
	midpoint = round(mean([startIdx, endIdx]));

	q = [quatRot.W(midpoint), quatRot.X(midpoint), quatRot.Y(midpoint), quatRot.Z(midpoint)];
	q = q/norm(q); % normalize quaternion
	w = q(1); x = q(2); y = q(3); z = q(4);
	R = [1-2*y^2-2*z^2, 2*x*y-2*z*w, 2*x*z+2*y*w;
     	2*x*y+2*z*w, 1-2*x^2-2*z^2, 2*y*z-2*x*w;
     	2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x^2-2*y^2];
	
	% Rotate axes
	rot_axes = R * (eye(3) * 10);
	
	set(quiverHandleX, 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,1), 'VData', rot_axes(2,1), 'WData', rot_axes(3,1));
	set(quiverHandleY, 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,2), 'VData', rot_axes(2,2), 'WData', rot_axes(3,2));
	set(quiverHandleZ, 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,3), 'VData', rot_axes(2,3), 'WData', rot_axes(3,3));

	% Update copies
	quivers2 = allchild(h2);
	set(quivers2(1), 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,1), 'VData', rot_axes(2,1), 'WData', rot_axes(3,1));
	set(quivers2(2), 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,2), 'VData', rot_axes(2,2), 'WData', rot_axes(3,2));
	set(quivers2(3), 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,3), 'VData', rot_axes(2,3), 'WData', rot_axes(3,3));
	
	quivers3 = allchild(h3);
	set(quivers3(1), 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,1), 'VData', rot_axes(2,1), 'WData', rot_axes(3,1));
	set(quivers3(2), 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,2), 'VData', rot_axes(2,2), 'WData', rot_axes(3,2));
	set(quivers3(3), 'XData', pos.X(midpoint), 'YData', pos.Y(midpoint), 'ZData', pos.Z(midpoint),...
		'UData', rot_axes(1,3), 'VData', rot_axes(2,3), 'WData', rot_axes(3,3));


	% Update speed plots
	set(plotHandleVel, 'XData', t(startIdx:endIdx), 'YData', rigidBodySpeed(startIdx:endIdx));
	set(plotHandleMarkSpeed, 'XData', t(startIdx:endIdx), 'YData', markerSpeedAvg(startIdx:endIdx));


    % Get the maximum absolute value across all data being plotted
    maxValPos = max(max([abs(pos.X(startIdx:endIdx)), abs(pos.Y(startIdx:endIdx)), abs(pos.Z(startIdx:endIdx))]));
    maxValEuler = max(max([abs(eulerUnwrap.roll(startIdx:endIdx)), abs(eulerUnwrap.pitch(startIdx:endIdx)), abs(eulerUnwrap.yaw(startIdx:endIdx))]));
    
    % Set Y-axis limits to include 0 and scale to the data
	subplot(4,3,1:2)
    ylim([-maxValPos, maxValPos]);
	subplot(4,3,4:5)
    ylim([-maxValEuler, maxValEuler]);
end




% SLERP (Spherical Linear intERPolation)
function q = slerp(q1, q2, smoothing)
    
    dot_product = dot(q1, q2);
    if dot_product < 0
        q1 = -q1;
        dot_product = -dot_product;
    end
    if dot_product > 0.9995
        q = q1 + smoothing*(q2 - q1);
    else
        theta_0 = acos(dot_product);
        sin_theta_0 = sin(theta_0);
        theta = theta_0 * smoothing;
        s1 = cos(theta) - dot_product * sin(theta) / sin_theta_0;
        s2 = sin(theta) / sin_theta_0;
        q = s1 * q1 + s2 * q2;
    end
end

