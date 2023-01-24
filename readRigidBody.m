function [motionImport] = readRigidBody(cfg)

% Function will read rigid body and marker data from a structure exported
% from OptiTrack Motive. It must only have one rigid body and only markers
% from that rigid body. It could be adapted to be more flexible though
% 
% Input:
% cfg.filename = string 'filename'
% cfg.importOrder = string 'xyz' or whatever was selected when exporting
% cfg.plot = true or false;
% the data from Motive. You can check the csv file.
%
% Output is a Matlab table:
% sample, time, rigid body cols, marker cols, where:
% rigid body cols = xPos, yPos, zPos, xOri, yOri, zOri, error
% marker cols = xPos, yPos, zPos, error. 

% Simplest way seems to be to read the file as a table first
motionImport = readtable(cfg.filename);

% Find first row that does not include only NaNs
firstNonNAN = find(~all(isnan(motionImport{:,:}),2),1);

% Set the table to just the rows after and including that
motionImport = motionImport(firstNonNAN:end,:);

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

% Rename columns for rigid body
columnNames(3:9) = [generalOrder {'error'}];

% Only keep the required columns.
colsToKeep = ~cellfun(@isempty, columnNames);
motionImport = motionImport(:,colsToKeep);

% Update the variable names
motionImport.Properties.VariableNames = columnNames(colsToKeep);

% Plot, if user specified
if cfg.plot
	figure;
	set(gcf,'Position',[1 1 1000 900]);
	subplot(3,1,1);
	plot(motionImport.Time,motionImport.posx,'r','LineWidth',2);
	set(gca,'FontSize',12);
	ylabel('Distance','FontSize',16);
	title('Rigid Body Position: X','FontSize',16);
	
	subplot(3,1,2);
	plot(motionImport.Time,motionImport.posy,'g','LineWidth',2);
	set(gca,'FontSize',12);
	ylabel('Distance','FontSize',16);
	title('Rigid Body Position: Y','FontSize',16);
	
	subplot(3,1,3);
	plot(motionImport.Time,motionImport.posz,'b','LineWidth',2);
	set(gca,'FontSize',12);
	ylabel('Distance','FontSize',16);
	title('Rigid Body Position: Z','FontSize',16);
	xlabel('Time (s)','FontSize',16);

	figure;
	set(gcf,'Position',[1 1 1000 900]);
	subplot(3,1,1);
	plot(motionImport.Time,motionImport.orix,'r','LineWidth',2);
	set(gca,'FontSize',12);
	ylabel('Angle','FontSize',16);
	title('Rigid Body Rotation: X','FontSize',16);
	
	subplot(3,1,2);
	plot(motionImport.Time,motionImport.oriy,'g','LineWidth',2);
	set(gca,'FontSize',12);
	ylabel('Angle','FontSize',16);
	title('Rigid Body Rotation: Y','FontSize',16);
	
	subplot(3,1,3);
	plot(motionImport.Time,motionImport.oriz,'b','LineWidth',2);
	set(gca,'FontSize',12);
	ylabel('Angle','FontSize',16);
	title('Rigid Body Rotation: Z','FontSize',16);
	xlabel('Time (s)','FontSize',16);
end



























