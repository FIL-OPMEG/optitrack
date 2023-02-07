function [linearVelocityMag,angularVelocityMag,spatialVelocityMag] = getRigidBodyVelocity(rigidBodyT, cfg)
% Function outputs the magnitude of linearVelocity, rotationVelocity and 
% spatialVelocity of a rigid body. Requires input of motionImport from 
% readRigidBody function and a cfg specification.
%
% motionImport = table from readRigidBody function. 
% 
% cfg.sg_filt = bool; % Apply a Savitzky-Golay filter (default = true).
% cfg.sg_freq = int; % Frequency of Savitzky-Golay filter (e.g. 2Hz = 0.5s)
% cfg.sg_order = int; % Order of Savitzky-Golay filter (default = 2).
% cfg.sampleRate = int; % fs, e.g. 120 or rigidBodyTables.cfg;
% cfg.plot = bool
% 
% Written by Nicholas Alexander (n.alexander@ucl.ac.uk) 31/01/2023

%% Set defaults
% Find the sample rate if not specified.
if ~isfield(cfg,'sampleRate')
	cfg.sampleRate = round(rigidBodyT.Frame(end)/rigidBodyT.Time(end));
	disp("Setting sample rate automatically. Sample rate = " + cfg.sampleRate + "Hz");
end

% Default Savitzky-Golay settings
if ~isfield(cfg,'sg_order')
	cfg.sg_order = 2;
	disp("Setting Savitzky-Golay filter order to default: " + cfg.sg_order);
end

if ~isfield(cfg,'sg_freq')
	cfg.sg_freq = 2;
	disp("Setting Savitzky-Golay filter order to default: " + cfg.sg_freq);
end

% Plot or not
if ~isfield(cfg.plot)
	cfg.plot = false;
end

%% Calculate the linear velocity
linearVelocity = zeros(length(rigidBodyT.Frame),3);
linearVelocityMag = zeros(length(rigidBodyT.Frame),1);
for i = 1:length(linearVelocity) - 1
	prevPos = [rigidBodyT.X_Position(i), rigidBodyT.Y_Position(i), rigidBodyT.Z_Position(i)];
	currPos = [rigidBodyT.X_Position(i+1), rigidBodyT.Y_Position(i+1), rigidBodyT.Z_Position(i+1)];
	linearVelocity(i,1:3) = currPos - prevPos;
	linearVelocityMag(i) = norm(linearVelocity(i));
end

%% Calculate the angular velocity
% Time data
dt = 1/cfg.sampleRate;

% Calculate derivatives
wx = gradient(deg2rad(rigidBodyT.X_Rotation), dt)./dt; % x angular velocity in rad/s
wy = gradient(deg2rad(rigidBodyT.Y_Rotation), dt)./dt; % y angular velocity in rad/s
wz = gradient(deg2rad(rigidBodyT.Z_Rotation), dt)./dt; % z angular velocity in rad/s

angularVelocity = [wx, wy, wz];

angularVelocityMag = zeros(length(rigidBodyT.Frame),1);
for i = 1:length(angularVelocity) - 1
	angularVelocityMag(i) = norm(angularVelocity(i+1) - angularVelocity(i));
end

%% Calculate the spatial velocity
% Find the linear velocity in three dimensions
pos = cumsum(linearVelocity,2)*dt; % Position (x, y, z) in meters
spatialVelocity = cross(angularVelocity,pos) + linearVelocity;

spatialVelocityMag = zeros(length(spatialVelocity),1);
for i = 1:length(linearVelocity) - 1
	spatialVelocityMag(i) = norm(spatialVelocity(i));
end


%% Clean up the velocities
% Savitzky-Golay filter the displacement, fitting a 2nd order polynomial to 2Hz*
if rem(cfg.sampleRate,2) == 0
	framelen = (cfg.sg_freq * cfg.sampleRate) + 1; 
else
	framelen = cfg.sg_freq * cfg.sampleRate; 
end

if cfg.sg_filter
	linearVelocityMag = sgolayfilt(linearVelocityMag, cfg.sg_order, framelen);
	angularVelocityMag = sgolayfilt(angularVelocityMag, cfg.sg_order, framelen);
	spatialVelocityMag = sgolayfilt(spatialVelocityMag, cfg.sg_order, framelen);

	% Can be an issue with negative values (impossible). Set to zero.
	zerosIdx = linearVelocityMag < 0;
	linearVelocityMag(zerosIdx) = 0;
	zerosIdx = angularVelocityMag < 0;
	angularVelocityMag(zerosIdx) = 0;
	zerosIdx = spatialVelocityMag < 0;
	spatialVelocityMag(zerosIdx) = 0;
end

%% Plot normalised outputs
if (cfg.plot)
	figure
	hold on
	title("Normalised magnitude of velocities")
	plot(rigidBodyT.Time, normaliseTimeseries(linearVelocityMag));
	plot(rigidBodyT.Time, normaliseTimeseries(angularVelocityMag));
	plot(rigidBodyT.Time, normaliseTimeseries(spatialVelocityMag));
	legend('Linear','Angular','Spatial')
else
	disp("Not plotting output")
end


end


function yNorm = normaliseTimeseries(y)
  % Subtract the mean from the time series
  y = y - mean(y);
  
  % Scale the time series to values between -1 and 1
  yNorm = 2*(y - min(y)) / (max(y) - min(y)) - 1;
end




