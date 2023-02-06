function [rigidBodyTables] = readRigidBody(cfg)
%__________________________________________________________________________
% Function will read rigid body and marker data from a structure exported
% from OptiTrack Motive. 
% 
% Input:
% cfg.filename = path
% cfg.plot = true or false;
%
% Output is a structure containing a field for each rigid body in the file.
% Those fields include the rigid body 6DOF info in one table and any
% rigid body marker data in another. At the top level there is another
% field containing a table containing any extra markers in the file, which
% may be repeats of the rigid body markers, depending on how the data were
% exported. There is a cfg structure containing info on the file. 
% 
% Authors:  Nicholas Alexander  (n.alexander@ucl.ac.uk)
%           Robert Seymour      (rob.seymour@ucl.ac.uk) 
%
% MIT License
%__________________________________________________________________________

%% Check cfg
if ~isfield(cfg, 'filename')
	error('You must specifiy a filename');
else
	if ~exist(cfg.filename,"file")
		error('The specified filename:\n%s\ndoes not exist',cfg.filename);
	end
end

% Remove anything else in the cfg
allowedFieldnames = {'filename'};
cfg = removeFields(cfg,allowedFieldnames);

%% Read in the file
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

% Get some useful information from the file
configOptions = {'Format Version','Take Name','Capture Frame Rate',...
	'Export Frame Rate','Capture Start Time','Total Frames in Take',...
	'Total Exported Frames','Rotation Type','Length Units','Coordinate Space'};

cfgString = strsplit(C{1},',');
for i = 1:length(configOptions)
	cfgIdx = contains(cfgString,configOptions{i});
	cfg.(regexprep(configOptions{i},' ','')) = cfgString{find(cfgIdx) + 1};
end

cfg = convertToDouble(cfg);

% Make the variable names from the rows
columnNames = combineStrings(C(objRow),C(nameRow),C(varRow),C(typeRow));

% Remove " and : and replace with a space
columnNames = regexprep(columnNames, '[":]', '');


% Read in the rest of the file, using the row after the row with "Frame" as column names
motionImport = readtable(cfg.filename, 'ReadVariableNames', false,'Range', dataRow);
motionImport.Properties.VariableNames = columnNames;

%% Organise into rigid bodies and markers
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

% Get the Frame and Time columns to duplicate across all tables
frameCol = 'Frame'; 
timeCol = 'Time';
frameColIdx = contains(columnNames, frameCol);
timeColIdx = contains(columnNames,timeCol);
commonTable = [motionImport(:,frameColIdx), motionImport(:,timeColIdx)];
commonTable.Properties.VariableNames = {frameCol, timeCol};
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
		rigidBodyTables.(uniqueRb{i}).(types{j}) = [commonTable, rigidBodyTables.(uniqueRb{i}).(types{j})];
	end
end

% Collect up any markers that are just on their own
cols = contains(columnNames,'_Marker_');
if any(cols)
	rigidBodyTables.RemainingMarkers = motionImport(:,cols);
	
	% Tidy this up too
	curCol = rigidBodyTables.RemainingMarkers.Properties.VariableNames;
	for i = 1:length(curCol)
		curCol{i} = regexprep(curCol{i}, '_Marker_', '');
		curCol{i} = regexprep(curCol{i}, '^_+', '');
		curCol{i} = regexprep(curCol{i}, '_+', '_');
	end
	rigidBodyTables.RemainingMarkers.Properties.VariableNames = curCol;
	rigidBodyTables.RemainingMarkers = [commonTable, rigidBodyTables.RemainingMarkers];
else
	disp("No additional markers found");
end

% Add in the cfgs
rigidBodyTables.cfg = cfg;

end

%% Subfunctions
% Method to combine multiple rows of comma delimited elements, accounting
% for some empty elements.
function combinedString = combineStrings(varargin)
    % Remove '_' that might have been used in naming conventions.
	varargin2 = cellfun(@(x) regexprep(x{1},'_',''), varargin, 'UniformOutput', false);

	% Split each string into cell arrays, preserving empty elements
	strings = cellfun(@(x) regexp(x, ',', 'split'), varargin2, 'UniformOutput', false);
    
    % Check that all strings have the same number of elements
    numElements = cellfun(@numel, strings);

	% If they don't try and make them all the same length by adding to the
	% beginning. Seems that Motive sometimes messes up their indexing and
	% shifts a row to the wrong position. 
    while numel(unique(numElements)) ~= 1
		warning("Your file may be a bit strange. Check that the row that defines a column type as rotation/position/Marker error is aligned correctly. This function may still work");
		[minElement, minIdx] = min(numElements);
		maxElement = max(numElements);
		strings{minIdx} = [cell(1,maxElement - minElement), strings{minIdx}];
		numElements = cellfun(@numel, strings);
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

% Remove unwanted inputs from a structure (i.e. cfg structure) so that
% nothing extra is passed to the output cfg. 
function structOut = removeFields(structIn, keepFields)
	structFields = fieldnames(structIn);
	removedFields = setdiff(structFields, keepFields);
	structOut = rmfield(structIn, removedFields);
	if ~isempty(removedFields)
		fprintf('The following fields were removed: %s\n', strjoin(removedFields, ', '))
	end
end

% Set fields to double if possible
function S = convertToDouble(S)
	fieldNames = fieldnames(S);
	for i = 1:numel(fieldNames)
		if ischar(S.(fieldNames{i}))
			value = str2double(S.(fieldNames{i}));
			if ~isnan(value) && value == round(value)
				S.(fieldNames{i}) = value;
			end
		end
	end
end


