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
        if ~ismember(fieldnames(MovementData), 'cfg')
            tinit = MovementData.time;
        else
            tinit = MovementData.RemainingMarkers.Time;
        end
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

if ~ismember(fieldnames(MovementData), 'cfg') % i.e. movement data extracted with csv2mat
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

else

    % Remaining Markers
    if mean(diff(tfinal)) > mean(diff(tinit))
        MovementData.RemainingMarkers{:,3:end} = filtfilt(B, A, table2array(MovementData.RemainingMarkers(:,3:end)));
    end
    newdat = interp1(tinit, MovementData.RemainingMarkers{:,[1,3:end]}, tfinal, method, 'extrap');
    MovementData_new.RemainingMarkers = table('Size',[size(newdat,1), size(MovementData.RemainingMarkers,2)],...
        'VariableTypes', varfun(@class,MovementData.RemainingMarkers,'OutputFormat','cell'), ...
        'VariableNames', MovementData.RemainingMarkers.Properties.VariableNames);
    MovementData_new.RemainingMarkers{:,[1,3:end]} = newdat;
    MovementData_new.RemainingMarkers{:,2} = tfinal';

    % Rigid Bodies
    rigid_body_names = setdiff(fieldnames(MovementData), {'RemainingMarkers', 'cfg'});
    for i = 1:length(rigid_body_names)
        
        % Filter
        if mean(diff(tfinal)) > mean(diff(tinit))
            MovementData.(rigid_body_names{i}).RigidBody{:,3:end} = filtfilt(B, A, table2array(MovementData.(rigid_body_names{i}).RigidBody(:,3:end)));
            MovementData.(rigid_body_names{i}).RigidBodyMarker{:,3:end} = filtfilt(B, A, table2array(MovementData.(rigid_body_names{i}).RigidBody(:,3:end)));
        end
        
        % Rigid body
        newdat = interp1(tinit, MovementData.(rigid_body_names{i}).RigidBody{:,[1,3:end]}, tfinal, method, 'extrap');
        MovementData_new.(rigid_body_names{i}).RigidBody = table('Size',[size(newdat,1), size(MovementData.(rigid_body_names{i}).RigidBody,2)],...
            'VariableTypes', varfun(@class,MovementData.(rigid_body_names{i}).RigidBody,'OutputFormat','cell'), ...
            'VariableNames', MovementData.(rigid_body_names{i}).RigidBody.Properties.VariableNames);
        MovementData_new.(rigid_body_names{i}).RigidBody{:,[1,3:end]} = newdat;
        MovementData_new.(rigid_body_names{i}).RigidBody{:,2} = tfinal';
    
        % Rigid body markers
        newdat = interp1(tinit, MovementData.(rigid_body_names{i}).RigidBodyMarker{:,[1,3:end]}, tfinal, method, 'extrap');
        MovementData_new.(rigid_body_names{i}).RigidBodyMarker = table('Size',[size(newdat,1), size(MovementData.(rigid_body_names{i}).RigidBodyMarker,2)],...
            'VariableTypes', varfun(@class,MovementData.(rigid_body_names{i}).RigidBodyMarker,'OutputFormat','cell'), ...
            'VariableNames', MovementData.(rigid_body_names{i}).RigidBodyMarker.Properties.VariableNames);
        MovementData_new.(rigid_body_names{i}).RigidBodyMarker{:,[1,3:end]} = newdat;
        MovementData_new.(rigid_body_names{i}).RigidBodyMarker{:,2} = tfinal';
    end
end

end

