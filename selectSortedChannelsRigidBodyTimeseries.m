function [channelRigidBodyTsorted] = selectSortedChannelsRigidBodyTimeseries(cfg)
% This function sorts the output of getChannelLevelRigidBodyTimeseries()
% based on the order of channels in the SPM object, using the mapping
% provided in the slot2sens.csv file. It discards any channel-level rigid
% body timeseries for which no matching sensors are found in the SPM
% object.
%
% % Example use:
% cfg						    = [];
% cfg.D                         = D;
% cfg.slot2sens                 = 'slot2sens.csv';
% cfg.channelLevelRbTimeseries  = channelLevelRbTimeseries;
% channelRigidBodyTsorted = selectSortedChannelsRigidBodyTimeseries(cfg);
%
% Author:	Sascha Woelk (s.wolk.16@ucl.ac.uk)
% MIT License

% Read slot2sens
slot2sens = readcell(cfg.slot2sens,"NumHeaderLines",1);

% Initialize an empty cell array for the complete list of channels
allChannelsSelected = {};

% Go through each sensor
chanIdx = 0;
chansD = cfg.D.chanlabels(cfg.D.indchantype('meg'));
channelRigidBodyT = cell(length(chansD),1);
for sensIdx = 1:length(slot2sens(:,1))

	% Find the labels in rawData that match the slot2sens
    pattern = strcat('(?<!\w)', slot2sens{sensIdx,2}, '(?!\d)(?!\w)');
	idx = find(~cellfun(@isempty, regexp(chansD, pattern)));
	channels = chansD(idx);

    % Append the channels to the complete list of channels
    allChannelsSelected = [allChannelsSelected(:); channels(:)];
    
    % Append rigid body data for all relevant channels (i.e. one per
    % orientation)
	for sensChanIdx = 1:length(channels)
		chanIdx = chanIdx + 1;
		if ismember(channels{sensChanIdx}(end),{'X','Y','Z'})
			channelRigidBodyT{chanIdx} = cfg.channelLevelRbTimeseries{slot2sens{sensIdx,1},2};
		else
			error("Cannot identify channel orientation")
		end
	end
end

% Assure the sorting of channel RigidBody Timeseries is correct
[~, sortIdx] = ismember(chansD, allChannelsSelected);
% Ensure there are no missing matches
if any(sortIdx == 0)
    error('Some channels in the SPM object are not found in the slot2sens file.');
end

% Sort channelLevelRbTimeseriesHead using the index
channelRigidBodyTsorted = channelRigidBodyT(sortIdx);