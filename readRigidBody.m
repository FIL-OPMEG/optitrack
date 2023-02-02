function [rigidBodyTables] = readRigidBody(cfg)
%__________________________________________________________________________
% Function will read rigid body and marker data from a structure exported
% from OptiTrack Motive. It must only have one rigid body and only markers
% from that rigid body. It could be adapted to be more flexible though.
% 
% Input:
% cfg.filename
% cfg.importOrder = string 'xyz' or whatever was selected when exporting
% cfg.plot = true or false;
% the data from Motive. You can check the csv file.
%
% Output is a Matlab table:
% sample, time, rigid body cols, marker cols, where:
% rigid body cols = xPos, yPos, zPos, xOri, yOri, zOri, error
% marker cols = xPos, yPos, zPos, error. 
% 
% Authors:  Nicholas Alexander  (n.alexander@ucl.ac.uk)
%           Robert Seymour      (rob.seymour@ucl.ac.uk) 
%
% MIT License
%__________________________________________________________________________

%% Set default values
if ~isfield(cfg, 'importOrder')
    cfg.importOrder = 'xyz';
end

if ~isfield(cfg, 'plot')
    cfg.plot = false;
end

%% Start of function proper
% Open the file
fid = fopen(cfg.filename);

% Read in the file line by line into a cell array
C = textscan(fid, '%s', 25, 'Delimiter', '\n');
C = C{1};

% Close the file
fclose(fid);

% Find the first instance of "Time (Seconds)" in the cell array
varRow = find(cellfun(@(x) contains(x, 'Time (Seconds)'), C), 1);

% Set the other useful rows
dataRow = varRow + 1;
typeRow = varRow - 1;
nameRow = varRow - 3;
objRow = varRow - 4;

% Make the variable names from the rows
columnNames = combineStrings(C(objRow),C(nameRow),C(varRow),C(typeRow));

% Remove " and : and replace with a space
columnNames = regexprep(columnNames, '[":]', '');


% Read in the rest of the file, using the row after the row with "Frame" as column names
motionImport = readtable(cfg.filename, 'ReadVariableNames', false,'Range', dataRow);
motionImport.Properties.VariableNames = columnNames;

%% Separate into rigid bodies and markers
% Identify the name and number of rigid bodies imported
% Get for strings after '_RigidBody_' in columnNames and check for unique
% ones.
possibleRb =  strsplit([columnNames{:}],'_RigidBody_');
possibleRb(1) = [];

for i = 1:length(possibleRb)
	tmp = strsplit(possibleRb{i},'_');
	possibleRb(i) = tmp(1);
end
uniqueRb = unique(possibleRb);

% Make a table for each rigid body
rigidBodyTables = [];
for i = 1:length(uniqueRb)
	types = {'RigidBody','RigidBodyMarker'};
	for j = 1:length(types)
		cols = contains(columnNames,strcat('_',types{j},'_',uniqueRb(i)));
		rigidBodyTables.(uniqueRb{i}).(types{j}) = motionImport(:,cols);

		% Tidy up column names
		curCol = rigidBodyTables.(uniqueRb{i}).(types{j}).Properties.VariableNames;
		for k = 1:length(curCol)
			curCol{k} = regexprep(curCol{k}, strcat('_',types{j},'_',uniqueRb(i)),'');
			curCol{k} = regexprep(curCol{k}, '^_+', '');
			curCol{k} = regexprep(curCol{k}, '_+', '_');
		end
		rigidBodyTables.(uniqueRb{i}).(types{j}).Properties.VariableNames = curCol;
	end
end

% Collect up any markers that are just on their own
cols = contains(columnNames,'_Marker_');
rigidBodyTables.RemainingMarkers = motionImport(:,cols);

% Tidy this up too
curCol = rigidBodyTables.RemainingMarkers.Properties.VariableNames;
for i = 1:length(curCol)
	curCol{i} = regexprep(curCol{i}, '_Marker_', '');
	curCol{i} = regexprep(curCol{i}, '^_+', '');
	curCol{i} = regexprep(curCol{i}, '_+', '_');
end
rigidBodyTables.RemainingMarkers.Properties.VariableNames = curCol;


%% Need to add plotting back in
% % Plot, if user specified
% if cfg.plot
% 	figure;
% 	set(gcf,'Position',[1 1 1000 900]);
% 	subplot(3,1,1);
% 	plot(motionImport.Time,motionImport.posx,'r','LineWidth',2);
% 	set(gca,'FontSize',12);
% 	ylabel('Distance','FontSize',16);
% 	title('Rigid Body Position: X','FontSize',16);
% 	
% 	subplot(3,1,2);
% 	plot(motionImport.Time,motionImport.posy,'g','LineWidth',2);
% 	set(gca,'FontSize',12);
% 	ylabel('Distance','FontSize',16);
% 	title('Rigid Body Position: Y','FontSize',16);
% 	
% 	subplot(3,1,3);
% 	plot(motionImport.Time,motionImport.posz,'b','LineWidth',2);
% 	set(gca,'FontSize',12);
% 	ylabel('Distance','FontSize',16);
% 	title('Rigid Body Position: Z','FontSize',16);
% 	xlabel('Time (s)','FontSize',16);
% 
% 	figure;
% 	set(gcf,'Position',[1 1 1000 900]);
% 	subplot(3,1,1);
% 	plot(motionImport.Time,motionImport.orix,'r','LineWidth',2);
% 	set(gca,'FontSize',12);
% 	ylabel('Angle','FontSize',16);
% 	title('Rigid Body Rotation: X','FontSize',16);
% 	
% 	subplot(3,1,2);
% 	plot(motionImport.Time,motionImport.oriy,'g','LineWidth',2);
% 	set(gca,'FontSize',12);
% 	ylabel('Angle','FontSize',16);
% 	title('Rigid Body Rotation: Y','FontSize',16);
% 	
% 	subplot(3,1,3);
% 	plot(motionImport.Time,motionImport.oriz,'b','LineWidth',2);
% 	set(gca,'FontSize',12);
% 	ylabel('Angle','FontSize',16);
% 	title('Rigid Body Rotation: Z','FontSize',16);
% 	xlabel('Time (s)','FontSize',16);
% end

end

function combinedString = combineStrings(varargin)
    % Split each string into cell arrays, preserving empty elements
    strings = cellfun(@(x) regexp(x{1}, ',', 'split'), varargin, 'UniformOutput', false);
    
    % Check that all strings have the same number of elements
    numElements = cellfun(@numel, strings);
    if numel(unique(numElements)) ~= 1
        error('All input strings must have the same number of elements');
    end
    
    % Concatenate the elements, preserving empty elements
    numElements = max(numElements);
    combinedString = cell(1, numElements);
    for i = 1:numElements
        combinedString{i} = '';
        for j = 1:numel(varargin)
            if i <= numel(strings{j})
                combinedString{i} = [combinedString{i} '_' strrep(strings{j}{i}, ' ', '')];
            end
        end
    end
    
    % Join the elements into a single string
    combinedString = strjoin(combinedString, ',');
	combinedString = regexp(combinedString, ',', 'split');
end






















