function [linearVelocityMagFilt,angularVelocityMagFilt,spatialVelocityMagFilt] = getRigidBodyVelocity(motionImport, cfg)
% Function outputs the magnitude of linearVelocity, rotationVelocity and 
% spatialVelocity of a rigid body. Requires input of motionImport from 
% readRigidBody function and a cfg specification.
%
% motionImport = table with columns: 
%				  Sample, Time, posx, posy, posz, orix, oriy, oriz, error 
% 
% cfg.sg_freq = int; % Frequency of Savitzky-Golay filter (e.g. 2Hz = 0.5s)
% cfg.sg_order = int; % Order of Savitzky-Golay filter (default = 2).
% cfg.sampleRate = int; % fs, e.g. 120;
% 
% Written by Nicholas Alexander (n.alexander@ucl.ac.uk) 31/01/2023

% Find the sample rate if not specified.
if isempty(cfg.sampleRate)
	cfg.sampleRate = round(motionImport.Sample(end)/motionImport.Time(end));
	disp("Setting sample rate automatically. Sample rate = " + cfg.sampleRate + "Hz");
end

%% Calculate the linear velocity
linearVelocity = zeros(length(motionImport.Sample),3);
linearVelocityMag = zeros(length(motionImport.Sample),1);
for i = 1:length(linearVelocity) - 1
	prevPos = [motionImport.posx(i), motionImport.posy(i), motionImport.posz(i)];
	currPos = [motionImport.posx(i+1), motionImport.posy(i+1), motionImport.posz(i+1)];
	linearVelocity(i,1:3) = currPos - prevPos;
	linearVelocityMag(i) = norm(linearVelocity(i));
end

%% Calculate the angular velocity
% Time data
dt = 1/sampleRate;

% Calculate derivatives
wx = gradient(deg2rad(motionImport.orix), dt)./dt; % x angular velocity in rad/s
wy = gradient(deg2rad(motionImport.oriy), dt)./dt; % y angular velocity in rad/s
wz = gradient(deg2rad(motionImport.oriz), dt)./dt; % z angular velocity in rad/s

angularVelocity = [wx, wy, wz];

angularVelocityMag = zeros(length(motionImport.Sample),1);
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
if rem(sampleRate,2) == 0
	framelen = (cfg.sg_freq * sampleRate) + 1; 
else
	framelen = cfg.sg_freq * sampleRate; 
end

linearVelocityMagFilt = sgolayfilt(linearVelocityMag, cfg.sg_order, framelen);
angularVelocityMagFilt = sgolayfilt(angularVelocityMag, cfg.sg_order, framelen);
spatialVelocityMagFilt = sgolayfilt(spatialVelocityMag, cfg.sg_order, framelen);

% Can be an issue with negative values (impossible). Set to zero.
zerosIdx = linearVelocityMagFilt < 0;
linearVelocityMagFilt(zerosIdx) = 0;
zerosIdx = angularVelocityMagFilt < 0;
angularVelocityMagFilt(zerosIdx) = 0;
zerosIdx = spatialVelocityMagFilt < 0;
spatialVelocityMagFilt(zerosIdx) = 0;

% % Plot them together
% figure
% hold on
% plot(motionImport.Time, linearVelocityMagFilt)
% plot(motionImport.Time, angularVelocityMagFilt)
% plot(motionImport.Time, spatialVelocityMagFilt)

% %% Compare output with 6DOF data
% data1 = linearVelocityMagFilt;
% data2 = motionImport.posx;
% data3 = motionImport.posy;
% data4 = motionImport.posz;
% 
% normData1 = (data1 - mean(data1)) / max(data1 - mean(data1));
% normData2 = (data2 - mean(data2)) / max(data2 - mean(data2));
% 
% 
% figure
% hold on
% plot(motionImport.Time, normData1)
% plot(motionImport.Time, normData2)








