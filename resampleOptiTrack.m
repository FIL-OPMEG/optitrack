function [ MovementData_new ] = resampleOptiTrack( MovementData, tfinal, tinit, method )
% Function to resample an OptiTrack object, created from csv2mat_sm
%
% INPUT:
%   - MovementData: the OptiTrack data you want to resample
%   - tfinial (double vector): new time vector in seconds
%   - tinit (optional double vector): original time vector in seconds. If
%       not included, MovementData.time will be used instead
%   - method (optional string or char): interpolation method. Default is
%       linear
%
% OUTPUT:
%   - MovementData_new: resampled OptiTrack data
%
% FUNCTION DEPENDENCIES:
%   - None
%
% AUTHOR:
%   Stephanie Mellor 31/3/20
%

if nargin < 4
    method = 'linear';
    if nargin < 3
        tinit = MovementData.time;
    end
end

MovementData_new = MovementData;

% Throw a warning message if tfinal and tinit are apart by more than 500 ms
if (max(tfinal) - max(tinit))^2 > 0.5 || (min(tfinal) - min(tinit))^2 > 0.5
    warning('Sampled times are not well matched; significant extrapolation will be required');
end

% Filter if dtfinal > dtinit
if mean(diff(tfinal)) > mean(diff(tinit))
    new_sampling_f = 1/mean(diff(tfinal));
    cutoff_f = new_sampling_f/2;
    sampling_f = 1/mean(diff(tinit));
    [B,A] = butter(3, 2*cutoff_f/sampling_f);
end

% Loop through different OptiTrack data types and resample

% - Rigid bodies
for i = 1:length(MovementData.rigidbodies)
    if mean(diff(tfinal)) > mean(diff(tinit))
        for j = 1:size(MovementData.rigidbodies(i).data, 2)
            MovementData.rigidbodies(i).data(:,j) = filtfilt(B, A, MovementData.rigidbodies(i).data(:,j));
        end
    end
    MovementData_new.rigidbodies(i).data = interp1(tinit, MovementData.rigidbodies(i).data, tfinal, method, 'extrap');
end

% - Rigid body markers
for i = 1:size(MovementData.markers.rigidbodymarkers,2)
    if mean(diff(tfinal)) > mean(diff(tinit))
        for j = 1:size(MovementData.markers.rigidbodymarkers(i).data, 2)
            MovementData.markers.rigidbodymarkers(i).data(:,j) = filtfilt(B, A, MovementData.markers.rigidbodymarkers(i).data(:,j));
        end
    end
    MovementData_new.markers.rigidbodymarkers(i).data = interp1(tinit, MovementData.markers.rigidbodymarkers(i).data, tfinal, method, 'extrap');
end

% - Labeled markers
for i = 1:size(MovementData.markers.labeledmarkers,2)
    if mean(diff(tfinal)) > mean(diff(tinit))
        for j = 1:size(MovementData.markers.labeledmarkers(i).data, 2)
            MovementData.markers.labeledmarkers(i).data(:,j) = filtfilt(B, A, MovementData.markers.labeledmarkers(i).data(:,j));
        end
    end
    MovementData_new.markers.labeledmarkers(i).data = interp1(tinit, MovementData.markers.labeledmarkers(i).data, tfinal, method, 'extrap');
end

MovementData_new.time = tfinal;

end

