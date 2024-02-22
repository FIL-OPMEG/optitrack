function [pos, euler] = processRigidBodyT(cfg,rigidBodyT)
% Function to plot rigid body positions.

% % Example usage
% % Read in the optitrack data
% cfg			= [];
% cfg.filename	= combined;
% rigidBodyT	= readRigidBody(cfg);

% % Plot
% cfg					= [];
% cfg.rigidBodyLabel	= 'Scannercast';
% cfg.lowpass			= 5;
% cfg.plot				= true;
% cfg.timeWindow		= 30;
% cfg.rotSmooth			= 0.3;
% cfg.unwrapRot			= true;
% [pos, euler] = plotRigidBodyT(cfg,rigidBodyT);

%% Start of function
% Filter design for positions
if (isfield(cfg,'lowpass') && ~isempty(cfg.lowpass))
	d = designfilt('lowpassiir', 'FilterOrder', 8, 'HalfPowerFrequency',cfg.lowpass,...
		'SampleRate', round(str2double(rigidBodyT.cfg.ExportFrameRate)));
else
	d = [];
end

%% Get position info (motive records Z = forward; Y = up, X = left)
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
end

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

%% Plot
if cfg.plot
	% Number of samples to plot at a time
	samplesToPlot = cfg.timeWindow * round(str2double(rigidBodyT.cfg.ExportFrameRate));
	
	% Time axis, starting at 0
	t = (1:length(pos.X)) / round(str2double(rigidBodyT.cfg.ExportFrameRate))...
		- 1/round(str2double(rigidBodyT.cfg.ExportFrameRate));
	
	% Create a new figure
	figure;
	
	% Create a slider
	slider = uicontrol('Style', 'slider',...
    	'Min', 1, 'Max', length(pos.X) - samplesToPlot + 1,...
    	'Value', 1, 'Position', [150 20 300 20]);
	
	%% Create a subplot for the position plot
	subplot(2,3,1:2)
	
	% Initial plot
	plotHandleX = plot(t(1:samplesToPlot), pos.X(1:samplesToPlot), 'Color', [1 0.2 0.2], 'LineWidth', 2); hold on;
	plotHandleY = plot(t(1:samplesToPlot), pos.Y(1:samplesToPlot),  'Color', [0.2 1 0.2], 'LineWidth', 2);
	plotHandleZ = plot(t(1:samplesToPlot), pos.Z(1:samplesToPlot), 'Color', [0.2 0.2 1], 'LineWidth', 2); hold off;
	
	xlabel("Time (s)")
	ylabel("Position (mm)")

	%% Create a subplot for the Euler plot
	subplot(2,3,4:5)
	
	% Initial plot for Euler angles
	plotHandleRoll = plot(t(1:samplesToPlot), eulerUnwrap.roll(1:samplesToPlot), 'Color', [1 0.2 0.2], 'LineWidth', 2); hold on;
	plotHandlePitch = plot(t(1:samplesToPlot), eulerUnwrap.pitch(1:samplesToPlot), 'Color', [0.2 1 0.2], 'LineWidth', 2);
	plotHandleYaw = plot(t(1:samplesToPlot), eulerUnwrap.yaw(1:samplesToPlot), 'Color', [0.2 0.2 1], 'LineWidth', 2); hold off;
	
	xlabel("Time (s)")
	ylabel("Euler Rotation (deg)")

	%% Create a subplot for the 3D plot
	subplot(2,3,[3,6])
	
	midpoint = round(mean([1, samplesToPlot]));

	% Define the origin
	origin = [pos.X(midpoint); pos.Y(midpoint); pos.Z(midpoint)];
	
	% Convert quaternion to rotation matrix
	% Assuming quaternion is in the form q = [w x y z]
	
	q = [quatRot.W(midpoint), quatRot.X(midpoint), quatRot.Y(midpoint), quatRot.Z(midpoint)];
	q = q/norm(q); % normalize quaternion
	w = q(1); x = q(2); y = q(3); z = q(4);
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

	% Create a listener for the slider
	addlistener(slider, 'Value', 'PostSet', @(src, event) updatePlot(slider, plotHandleX, plotHandleY, plotHandleZ, ...
		plotHandleRoll, plotHandlePitch, plotHandleYaw, quiverHandleX, quiverHandleY, quiverHandleZ,...
		samplesToPlot, pos, eulerUnwrap, t, quatRot));
	
	% Initial update
	updatePlot(slider, plotHandleX, plotHandleY, plotHandleZ, ...
		plotHandleRoll, plotHandlePitch, plotHandleYaw, quiverHandleX, quiverHandleY, quiverHandleZ,...
		samplesToPlot, pos, eulerUnwrap, t, quatRot)
end

if (isfield(cfg,'unwrapRot') && cfg.unwrapRot)
	euler = eulerUnwrap;
end


end

function updatePlot(slider, plotHandleX, plotHandleY, plotHandleZ, ...
		plotHandleRoll, plotHandlePitch, plotHandleYaw, quiverHandleX, quiverHandleY, quiverHandleZ,...
		samplesToPlot, pos, eulerUnwrap, t, quatRot)

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

    % Get the maximum absolute value across all data being plotted
    maxValPos = max(max([abs(pos.X(startIdx:endIdx)), abs(pos.Y(startIdx:endIdx)), abs(pos.Z(startIdx:endIdx))]));
    maxValEuler = max(max([abs(eulerUnwrap.roll(startIdx:endIdx)), abs(eulerUnwrap.pitch(startIdx:endIdx)), abs(eulerUnwrap.yaw(startIdx:endIdx))]));
    
    % Set Y-axis limits to include 0 and scale to the data
	subplot(2,3,1:2)
    ylim([-maxValPos, maxValPos]);
	subplot(2,3,4:5)
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

