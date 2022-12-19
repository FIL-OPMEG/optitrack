% function [

% Function will read rigid body and marker data from a structure exported
% from OptiTrack Motive. It must only have one rigid body and only markers
% from that rigid body. It could be adapted to be more flexible though
% 
% Input:
% cfg.filename = string 'filename'
% cfg.importOrder = string 'xyz' or whatever was selected when exporting
% the data from Motive. You can check the csv file.
%
% Output is a Matlab table:
% sample, time, rigid body cols, marker cols, where:
% rigid body cols = xPos, yPos, zPos, xOri, yOri, zOri, error
% marker cols = xPos, yPos, zPos, error. 

% Debug input:
cfg.importOrder = 'XYZ';

% Simplest way seems to be to read the file as a table first
motionImport = readtable(motionFilename);

% Remove nans
motionImport = motionImport(~any(ismissing(motionImport),2),:);

% Get the automatically labelled columns
autoColumnNames = motionImport.Properties.VariableNames;

% Create a new column name cell array
columnNames = cell(size(autoColumnNames));

% First two are samples and time
columnNames(1:2) = [{'Sample'}, {'Time'}];

% Take the user input and split it into three.
order = num2cell(cfg.importOrder);

% Apply the user specified order to the general expected order
generalOrder = [{'pos'},{'pos'},{'pos'},{'ori'},{'ori'},{'ori'}];

orderCount = 0;
for i = 1:length(generalOrder)
	generalOrder{i} = [generalOrder{i},order{rem(orderCount,3) + 1}];
	orderCount = orderCount + 1;
end

% Reference the rigid body and marker columbs
rigidBodyIdx = zeros(size(autoColumnNames));
markerIdx = rigidBodyIdx;
for colIdx = 1:length(rigidBodyIdx)
	col = autoColumnNames{colIdx};
	mightBeRb = contains(col,'RigidBody');
	isMarker = contains(col,'RigidBodyMarker');
	rigidBodyIdx(colIdx) = and(xor(mightBeRb,isMarker),mightBeRb);
	markerIdx(colIdx) = isMarker;
end

% And returning an error if there are more than 7.
if sum(rigidBodyIdx) ~= 7
	error("Imported data must contain exactly one rigid body");
else
	disp('One rigid body identified');
end

% Count the number of markers and let the user know
markerCount = sum(markerIdx)/4;
disp(strcat(num2str(markerCount),' rigid body markers identified'));

% Rename columns for rigid body
columnNames(3:9) = [generalOrder {'error'}];

% TO DO: We don't really need markers so only add that in if needed.

% Make a final table



































