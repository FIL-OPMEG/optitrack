function [ MovementDataOut, OPMdataOut ] = syncOptitrackAndOPMdata( MovementData, OPMdata, varargin )
% Synchronise OPM and Optitrack recordings
% Trims and resamples each dataset. Should occur before epoching.
%
% The trigger is asssumed to be in the same time as the OPM data and by
% default the OptiTrack data is resampled to match the OPMs, rather than 
% the other way around, although this can be adjusted using the
% "ResampleOPMdata" input option.
%
%   Input Options:
%       syncOptitrackAndOPMdata(MovementData, D)
%       syncOptitrackAndOPMdata(MovementData, ft_OPMdata)
%       syncOptitrackAndOPMdata(MovementData, OPMdataMatrix)
%       syncOptitrackAndOPMdata(___, 'LengthsAlreadyMatch', false)
%       syncOptitrackAndOPMdata(___, 'TriggerChannelName', 'OptiTrig')
%       syncOptitrackAndOPMdata(___, 'Trigger', TriggerVector)
%       syncOptitrackAndOPMdata(___, 'ResampleOPMdata', false)
%
% INTPUT:
%   - MovementData: the OptiTrack recordings. Can be either a struct object
%       with elements for Time, RigidBodies, Markers etc. (as output by
%       csv2mat_sm) or a Matrix with a column for each vector you would
%       like to sync with the OPM data. In the latter case, the lengths of
%       the OPM and Optitrack data must either already match (so there is
%       no need to trim either dataset) or the full trigger (off, on, off)
%       must be available, i.e. off then on is not an option without a time
%       variable in the MovementData.
%   - OPMdata options:
%       - D: SPM meeg object containing OPM data. If this option is used, SPM12
%           must be installed on the PC and in the matlab file path.
%       - ft_OPMdata: OPM data as arranged by fieldtrip. If this option is
%           used, FieldTrip must be installed and in the matlab file path.
%       - OPMdataMatrix: Alternatively, the OPM data can be input as a matrix
%           with dimension order channel_time. In this case, either
%           LengthsAlreadyMatch must be set to true or the Trigger must be
%           explicitly provided and contain the full (off, on, off) cycle.
%   - Additional Options - must be entered key followed by value:
%       - LengthsAlreadyMatch (default: false): Boolean to determine
%           whether or not to trim the OptiTrack data to match a given
%           trigger. When set to true, it is assumed that the start and end
%           of the OptiTrack and OPM recordings are already syncronised so
%           there so any trimming of the data will be skipped. 
%       - TriggerChannelName (default: 'OptiTrig'): If trimming of the data
%           is required and the OPM data is a meeg or FieldTrip object, you
%           can specify the name of the trigger channel here. This trigger
%           must be high when the OptiTrack is recording and low when it is 
%           off.
%       - Trigger: Double array of zeros and ones for when the Optitrack is
%           recording (1) or off (0). Particularly useful way to enter the
%           Optitrack trigger if you either put the OPM data in as a matrix
%           or recorded triggers in such a way that it wasn't a
%           straightforward high-gated (optitrack on when high) trigger.
%           N.B. the assumption is made that if only one change in the
%           trigger is found, then this is the Optitrack turning on and the
%           moment it turns off was missed by the OPMs, so the Movement
%           data is also trimmed. A warning will be produced if this is the
%           case.
%       - ResampleOPMdata (default: false): Boolean to choose to resample
%           the OPM data to match the OptiTrack data, rather than the other
%           way around. All of the trimming will still match the previously
%           given pattern (i.e. trim the OPM data unless only one trigger
%           point is available).
%
% OUTPUT:
%   - MovementDataOut: Resampled (or untouched if ResampleOPMdata = true) 
%       OptiTrack recordings, output in the same format as they are input
%   - OPMdataOut: Trimmed and/or resampled OPM data, output in the same
%       format as it is input
%
% FUNCTION DEPENDENCIES:
%   - resampleOptiTrack (if you've input the MovementData as a struct)
%
% AUTHOR:
%   Stephanie Mellor, UCL (stephanie.mellor.17@ucl.ac.uk)


% Parse inputs
defaults = struct('LengthsAlreadyMatch', false, ...
    'TriggerChannelName', 'OptiTrig', 'ResampleOPMdata', false);  % define default values
params = struct(varargin{:});
for f = fieldnames(defaults)'
    if ~isfield(params, f{1})
        params.(f{1}) = defaults.(f{1});
    end
end
clear defaults

% Check type of OPM data entered
if isa(OPMdata, 'meeg')
    OPMdataType = 'spm';
elseif isstruct(OPMdata) && (isfield(OPMdata, 'trial') || isfield(OPMdata, 'avg'))
    OPMdataType = 'ft';
elseif isa(OPMdata, 'double')
    OPMdataType = 'matrix';
else
    error('Invalid OPM Data Type given');
end

% Check type of Movement Data entered
if isstruct(MovementData)
    MovementDataType = 'struct';
    % Check that as a minimum, rigidbodies or markers fields exist
    if ~(isfield(MovementData, 'rigidbodies') || isfield(MovementData, 'markers'))
        error('Movement Data must have fields for at least 1 of rigid bodies or markers');
    end
elseif isa(MovementData, 'double')
    MovementDataType = 'matrix';
else
    error('Invalid Movement Data Type given');
end

% Trim data
if ~params.LengthsAlreadyMatch
    % Get Trigger channel
    if ~isfield(params, 'Trigger')
        switch OPMdataType
            case 'spm'
                [~,~,optiChanind] = intersect(params.TriggerChannelName, OPMdata.chanlabels);
                params.Trigger = OPMdata(optiChanind, :, :);
            case 'ft'
                    pos_of_trig     = ismember(OPMdata.label,params.TriggerChannelName);
                    params.Trigger  = OPMdata.trial{1}(pos_of_trig,:);
            case 'matrix'
                error(sprintf('When the OPM data is provided as a matrix, either the length of the data should already\n be set to match the Optitrack data, in which case LengthsAlreadyMatch should\n be set to true, or the trigger must be explicitly provided.'));
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RS note to self: This needs to be done better
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     % Find edges of trigger (where it steps up and steps down)
     [~, samples] = findpeaks(abs(diff(params.Trigger)), 'MinPeakHeight', 0.5*max(abs(diff(params.Trigger)))); 
      
    % Check length of samples is 2. If only one step was recorded, produce 
    % a warning then assume it was the start. Otherwise raise error
    if length(samples) > 2
        error('Too many steps in the trigger were found. Check the trigger is only on for one time period.');
    elseif length(samples) == 0
        error('No trigger steps found. Check given trigger channel.');
    elseif length(samples) == 1
        warning('Only one step in the trigger was found. It will be assume that this is from the start of the Optitrack recording');
    elseif length(samples) == 2
        switch OPMdataType
            case 'ft'
                disp(['Start-End of OPM Trigger: ' num2str(round(...
                    OPMdata.time{1}(samples(2)) - OPMdata.time{1}(samples(1)),2))...
                    's']);
            case 'spm'
                disp(['Start-End of OPM Trigger: ' num2str(round(...
                    OPMdata.time(samples(2)) - OPMdata.time(samples(1)),2))...
                    's']);
        end
     disp(['Length of Optitrack data: ' num2str(round(...
         MovementData.time(end),2)) 's']);
    end
    
    % Trim OPM data
    switch OPMdataType
        case 'spm'
            if length(samples) == 2
                Dnew = clone(OPMdata, ['t_', fname(OPMdata)], [OPMdata.nchannels, length(samples(1):samples(2)), 1]);
                Dnew = timeonset(Dnew, 0);
                Dnew(:,:,:) = OPMdata(:,samples(1):samples(2),:);
                OPMdataOut = Dnew;
            elseif length(samples) == 1
                Dnew = clone(OPMdata, ['t_', fname(OPMdata)], [OPMdata.nchannels, length(samples(1):OPMdata.nsamples), 1]);
                Dnew = timeonset(Dnew, 0);
                Dnew(:,:,:) = OPMdata(:,samples(1):OPMdata.nsamples,:);
                OPMdataOut = Dnew;
            end
            clear Dnew
            time = OPMdataOut.time;
        case 'ft'
            cfg = [];
            time = OPMdata.time{1};
            if length(samples) == 2
                cfg.latency = [time(samples(1)), time(samples(2))];
            elseif length(samples) == 1
                cfg.latency = [time(samples(1)), time(end)];
            end
            OPMdataOut = ft_selectdata(cfg, OPMdata);
            time = OPMdataOut.time{1};
        case 'matrix'
            if length(samples) == 2
                OPMdataOut = OPMdata(:,samples(1):samples(2),:);
            elseif length(samples) == 1
                OPMdataOut = OPMdata(:,samples(1):end,:);
            end
    end
    
    % Trim movement data (if needed)
    if length(samples) == 1
        switch MovementDataType
            case 'struct'
                if exist('time', 'var')
                    t1 = time;
                else
                    error('If the OPMdata is provided as a matrix, the full trigger cycle (off, on, off) must be provided');
                end
                t0 = MovementData.time;
                trim_idxs = (t0 > max(t1));
                
                % - Labeled markers
                for i = 1:size(MovementData.markers.labeledmarkers,2)
                    MovementData.markers.labeledmarkers(i).data(trim_idxs,:) = [];
                end
                % - Rigid body markers
                for i = 1:size(MovementData.markers.rigidbodymarkers,2)
                    MovementData.markers.rigidbodymarkers(i).data(trim_idxs,:) = [];
                end
                MovementData.time(trim_idxs) = [];
                
                clear t0 t1 
                
            case 'matrix'
                error(sprintf('Full trigger was not found. When the movement data is given as a matrix, either the movement\nand OPM data must already start and end at the same time, or the full trigger must be given.'));
        end
    end
end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RS note to self: There needs to be a check here whether the times
    % match the sampling rate of the optitrack and OPM data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% Resample Data
if ~params.ResampleOPMdata
    switch MovementDataType
        case 'struct'
            % Get time vector you want to match to:
            switch OPMdataType
                case 'spm'
                    t1 = OPMdataOut.time;
                case 'ft'
                    tinit   = OPMdataOut.time{1};
                    t1  = tinit-tinit(1);
                case 'matrix'
                    % Given ends of MovementData and OPMdata times must
                    % match, create time variable from number of samples in
                    % OPMdata and time vector from MovementData
                    t1 = linspace(MovementData.time(1), MovementData.time(end), size(OPMdataOut,2));
            end
            MovementDataOut = resampleOptiTrack(MovementData,t1);
            if strcmp(OPMdataType, 'ft')
                MovementDataOut.time = OPMdataOut.time{1};
            end
        case 'matrix'
            switch OPMdataType
                case 'spm'
                    t1 = OPMdataOut.time;
                case 'ft'
                    t1 = OPMdataOut.time{1};
                case 'matrix'
                    t1 = linspace(0, 1, size(OPMdataOut,2));                    
            end
            t0 = linspace(min(t1), max(t1), size(MovementData, 1));
            MovementDataOut = interp1(t0, MovementData, t1);
    end
    
    % Raise warning if downsampling as data will be filtered with
    % Butterworth filter
    if 1/mean(diff(t1)) < 1/mean(diff(MovementData.time))
        warning(sprintf('You are downsampling the Movement Data so the data will be filtered to reduce aliasing.'));
    end
else
    MovementDataOut = MovementData;
    % Find the vector you're resampling to
    switch MovementDataType
        case 'struct'
            t1 = MovementData.time;
        case 'matrix'
            t1 = linspace(0, 1, size(MovementData,1));
    end
            
    switch OPMdataType
        case 'spm'
            
            % Find the frequency you're resampling to
            nMovementSamples = length(t1);
            nOPMsamples = size(OPMdataOut,2);
            f1 = OPMdataOut.fsample;
            f2 = f1*nMovementSamples/nOPMsamples;
            
            % Use spm's inbuilt downsample function
            S = [];
            S.D = OPMdataOut;
            S.fsample_new = f2;
            OPMdataOut = spm_eeg_downsample(S);
            
            % Just check the sizes match
            if size(OPMdataOut,2) ~= nMovementSamples
                error('Bug needs fixing - this should not happen so raise if it does');
            end
            
        case 'ft'
            
            % Find frequency you're resampling to
            nMovementSamples = length(t1);
            nOPMsamples = length(OPMdataOut.time{1});
            f1 = 1/mean(diff(OPMdataOut.time{1}));
            f2 = f1*nMovementSamples/nOPMsamples;
            
            % Use fieldtrip's inbuilt resample data function
            cfg = [];
            cfg.resamplefs = f2;
            cfg.detrend = 'no';
            OPMdataOut = ft_resampledata(cfg, OPMdataOut);
            
            % Just check sizes match
            if length(OPMdataOut.time{1}) ~= nMovementSamples
                error('Bug needs fixing - this should not happen so raise if it does');
            end
            
        case 'matrix'
            
            % Find time vector you're resampling from
            t0 = linspace(min(t1), max(t1), size(OPMdataOut, 2));
            
            % Currently no filtering implemented so raise warning if downsampling.
            if 1/mean(diff(t1)) < 1/mean(diff(MovementData.time))
                warning(sprintf('No filtering is implemented in syncOptitrackAndOPMdata.\nYou are downsampling the OPM Data so this may cause aliasing.'));
            end
            
            % Use interp1 to resample
            OPMdataOut = transpose(interp1(t0, OPMdataOut', t1));
    end
    
end

fprintf('OPM and OptiTrack data should now be synchronised.\n');


end

