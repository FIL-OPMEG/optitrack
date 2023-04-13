function [MovementDataOut, OPMdataOut ] = ft_syncOptitrackOPM(cfg, MovementData, OPMdata)
%__________________________________________________________________________
% ft_syncOptitrackOPM synchronises OPM and Optitrack recordings
% Trims and resamples each dataset. Should occur before epoching.
%
% Use as
%   MovementDataOut, OPMdataOut = 
%                          ft_syncOptitrackOPM(cfg, MovementData, OPMdata)
% where the MovementData input argument comes from readRigidBody and 
% OPMdata is a raw Fieldtrip data structure.
%
% cfg is a configuration structure that should contain
%   cfg.trigger     = logical array corresponding to on/off times of the
%                     mocap recording
%
% In addition the following cfg options are supported:
%   cfg.rigidbody   = cell array of rigid body names to process. If not
%                     included all are processed
%   cfg.resamplefs  = resampling frequency required, if different from the
%                     neuroimaging data
%   cfg.trimOPMdata = do you want to trim the OPM data? (default = 'yes')
%   cfg.method      = method for upsampling (see MATLAB documentation of
%                     interp1 for more info)
%
% Authors:  Robert Seymour, 2023    (rob.seymour@ucl.ac.uk) 
%           (built on original code by Steph Mellor, UCL)
%
% MIT License
%__________________________________________________________________________

%% Set the defaults
if ~isfield(cfg, 'trigger')
    ft_warning(['Please provide a trigger!']);
    return
end

if ~isfield(cfg, 'trimOPMdata')
    cfg.trimOPMdata = 'yes';
end

if ~isfield(cfg, 'resamplefs')
    cfg.resamplefs = OPMdata.fsample;
end

if ~isfield(cfg, 'method')
    cfg.method = 'linear';
end

%% Check trigger is valid
trigger = cfg.trigger;

if islogical(trigger)
    % Check size
    if length(trigger) ~= size(OPMdata.trial{1},2)
        error('Length of trigger and data need to be identical...');
    end

    % Check for only two change points
    if ~sum(diff(trigger) == 1) == 1
        error('Trigger needs to go ON (i.e. change from 0-1) ONLY once');
    end
    
%% If user has specified an array of data try to extract a trigger
else
    ft_warning('Only logical triggers fully supported for the moment');
   
    % Below is a work-in-progress option for the user to use a trigger
    % channel in their data

%     % Check size
%     if length(trigger) ~= size(OPMdata.trial{1},2)
%         error('Length of trigger and data need to be identical...');
%     end

    % Find edges of trigger (where it steps up and steps down)
     [~, samples] = findpeaks(abs(diff(cfg.trigger)), ...
         'MinPeakHeight', 0.5*max(abs(diff(cfg.trigger)))); 
      
    % Check length of samples is 2. If only one step was recorded, produce 
    % a warning then assume it was the start. If there are 4 samples 
    % assume this corresponds to the onset of two triggers. Otherwise raise
    % an error.

    if length(samples) > 5
        error(['Too many steps in the trigger were found. ' ...
            'Check the trigger is only on for one time period.']);
    elseif length(samples) == 0
        error('No trigger steps found. Check given trigger channel.');
    elseif length(samples) == 1
        warning(['Only one step in the trigger was found. It will be'...
            'assume that this is from the start of the Optitrack ' ...
            'recording']);

        % Create logical array
        cfg.trigger = zeros(1,length(cfg.trigger));
        cfg.trigger(samples(1):end) = 1;
        cfg.trigger = logical(cfg.trigger);

    elseif length(samples) == 2

        % Create logical array
        trigger = zeros(1,length(cfg.trigger));
        trigger(samples(1):samples(2)) = 1;
        trigger = logical(trigger);
        
    elseif length(samples) == 4
        % This is a very custom thing for Nic + Rob' Filbury thing
        warning(['Two triggers found. Using the onset of the first and'...
            'last. Proceed with caution']);
        % Create logical array
        trigger = zeros(1,length(cfg.trigger));
        trigger(samples(1):samples(3)) = 1;
        trigger = logical(trigger);
    end

%% Resample OPM data if required
if cfg.resamplefs ~= OPMdata.fsample
    disp(['Resampling the OPM data to ' num2str(OPMdata.fsample) 'Hz']);
    cfg2                = [];
    cfg2.resamplefs     = cfg.resamplefs;
    OPMdata             = ft_resampledata(cfg,OPMdata);
end

%% Get list of fields
if ~isfield(cfg,'rigidbody')
    fields = fieldnames(MovementData);
    fields = fields(~strcmp(fields,'cfg'));
else
    fields = cfg.rigidbody;
end

%% Get time
tfinal = OPMdata.time{1};
tfinal = tfinal(trigger);
tfinal = tfinal-tfinal(1);

%% Resample the Movement Data
disp(['Resampling the mocap data to ' num2str(cfg.resamplefs) 'Hz'])
MovementDataOut = [];

% For each field
for f = 1:length(fields)
    % Get a list of sub-fields
    subfields = fieldnames(MovementData.(fields{f}));
    % For each subfield (e.g. RigidBody, RigidBodyMarker)
    for sf = 1:length(subfields)

        % Display message for resampling data
        disp(' ');
        disp(['Resampling: ' (fields{f}) '.' (subfields{sf})]);

        % Get correct data
        
        rgb = MovementData.(fields{f}).(subfields{sf});
        
        % Get initial time
        tinit = rgb.Time;

        % Display timings
        difference = abs(tinit(end) - tfinal(end));
        disp(['Trigger length: ' num2str(tfinal(end)) 's. ' ...
            fields{f} ' ' (subfields{sf}) ' length: ' num2str(tinit(end))...
            's. Difference: ' num2str(difference) 's.']);

        % Through a warning message if tfinal and tinit are apart by more than 500 ms
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

        % Make a new table
        rgb_new = table;

        % Get column names
        field2  = rgb.Properties.VariableNames;

        % For each column
        for ff = 1:length(field2)
            dat = rgb.(field2{ff});
            
            % Filter if dtfinal > dtinit
            if mean(diff(tfinal)) > mean(diff(tinit))
                dat = filtfilt(B, A, dat);
            end

            % Use interp1 to upsample
            dat_out = interp1(tinit, dat, tfinal, cfg.method, 'extrap');
            rgb_new.(field2{ff}) = dat_out';
        end

        % Replace frame with integers
        try
            rgb_new.Frame = [0:size(rgb_new,1)-1]';
        catch
        end

        % Create a new structure
        MovementDataOut.(fields{f}).(subfields{sf}) = rgb_new;

        clear rgb_new rgb dat_out

    end
end

% Reformat the cfg info
MovementDataOut.cfg                              = MovementData.cfg;
MovementDataOut.cfg.UpSampledCaptureFrameRate    = cfg.resamplefs;
MovementDataOut.cfg.UpSampledExportFrameRate     = cfg.resamplefs;
MovementDataOut.cfg.UpSampledTotalFramesinTake   = length(tfinal);
MovementDataOut.cfg.UpSampledTotalExportedFrames = length(tfinal);
    
%% Trim the OPM data to trigger
if ~strcmp(cfg.trimOPMdata,'no')
    t_trig = OPMdata.time{1};
    t_trig = t_trig(trigger);

    disp(' ');
    disp(['Trimming the OPM data between ' num2str(t_trig(1)) 's - ' ...
        num2str(t_trig(end)) 's'])

    % Use ft_selectdata to do this
    cfg2                 = [];
    cfg2.latency         = [t_trig(1) t_trig(end)];
    cfg2.keepsampleinfo  = 'yes';
    OPMdataOut          = ft_selectdata(cfg2,OPMdata);
else
    OPMdataOut = OPMdata;
end

%% Display message: DONE!
disp('');
disp('COMPLETED!');

end

