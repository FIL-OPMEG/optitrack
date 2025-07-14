function [grad] = updateSensorPositionsFrame(grad, sensorLevelRbTimeseries, sampleIdx)
% Helper function to update the OPM channel positions and orientations for
% a given moment in time, using the movement data of a corresponding rigid 
% body object from an Optitrack recording.
% 
% INPUT:
%   - grad: grad structure from the SPM obejct, containing only the
%   relevant sensors.
%   - sensorLevelRbTimeseries: movement data, extracted from the
%   Optitrack rigid body, sorted in the same order as the grad structure.
%   - sampleIdx: the sample point corresponding to the moment in time for
%   which the position and orientation of the sensos are required.
%
% OUTPUT:
%   - grad: grad structure with updated channel positions and orientations
%
% % Example use:
% % (see individual functions for cfg specifications)
%
% [sensorLevelRbTimeseries] = getChannelLevelRigidBodyTimeseries(cfg);
% cD = spm_eeg_crop(cfg);
% grad = extractGradSelectedSensors(cD);
% grad = updateSensorPositionsFrame(grad, sensorLevelRbTimeseries, sampleIdx);
%
% Author:	Sascha Woelk (s.wolk.16@ucl.ac.uk)
% MIT License

% Update the coil positions based on the rigid body at the specified frame
grad.coilpos(:,1) = cellfun(@(tbl) tbl.X_Position(sampleIdx), sensorLevelRbTimeseries);
grad.coilpos(:,2) = cellfun(@(tbl) tbl.Y_Position(sampleIdx), sensorLevelRbTimeseries);
grad.coilpos(:,3) = cellfun(@(tbl) tbl.Z_Position(sampleIdx), sensorLevelRbTimeseries);

% Update the channel positions based on the rigid body at the specified frame
grad.chanpos(:,1) = cellfun(@(tbl) tbl.X_Position(sampleIdx), sensorLevelRbTimeseries);
grad.chanpos(:,2) = cellfun(@(tbl) tbl.Y_Position(sampleIdx), sensorLevelRbTimeseries);
grad.chanpos(:,3) = cellfun(@(tbl) tbl.Z_Position(sampleIdx), sensorLevelRbTimeseries);

% Update the coil and channel orientations
for h = 1:height(sensorLevelRbTimeseries)

    % Extract the quaternion components from the rigid body at the specified frame
    quat_w = sensorLevelRbTimeseries{h}.W_Rotation(sampleIdx);
    quat_x = sensorLevelRbTimeseries{h}.X_Rotation(sampleIdx);
    quat_y = sensorLevelRbTimeseries{h}.Y_Rotation(sampleIdx);
    quat_z = sensorLevelRbTimeseries{h}.Z_Rotation(sampleIdx);

    % Convert quaternions to rotation matrix
    sensR = quat2rotm([quat_w, quat_x, quat_y, quat_z]);

    % Find the axis type of the current sensor
    switch true
        case endsWith(grad.label{h}, '-X')
            sensAxis = 1;
        case endsWith(grad.label{h}, '-Y')
            sensAxis = 2;
        case endsWith(grad.label{h}, '-Z')
            sensAxis = 3;
        otherwise
            error('Unexpected label suffix.');
    end

    % Update the coil and channel orientations
    if sensAxis == 1
        grad.coilori(h, :) = -sensR(:,sensAxis)';
        grad.chanori(h, :) = -sensR(:,sensAxis)';
    else
        grad.coilori(h, :) = sensR(:,sensAxis)';
        grad.chanori(h, :) = sensR(:,sensAxis)';
    end

end