function OTobj = csv2mat_sm(filename)
% function OTobj = csv2mat_sm(filename)
% Function to convert an OptiTrack csv to a structure containing the header
%   information and marker and rigid object positions
%
% INPUT:
%   filename (string) - name of the csv file you want to convert. Must be outputed
%       from Motive and include the full filename, including directory and
%       file extension
%
% OUPUT:
%   OTobj (struct) - structure with fields takeName, captureFrameRate, 
%       exportFrameRate, startTime, totalFrames, lengthUnits,
%       markers, rigidBodies
%
% AUTHOR:
%   Stephanie Mellor (stephanie.mellor.17@ucl.ac.uk)
%   Based on csv2mat from Juan Pablo Angel Lopez https://uk.mathworks.com/matlabcentral/fileexchange/63721-csv-files-processing-for-mocap-with-optitrack

FullFile = textread(filename, '%s', 2, 'delimiter', ',');
OTobj.filename = filename;

if strcmp(FullFile{2}, '1.22')
    % Get setup info
    [~, ~, ~, OTobj.takeName, ~, OTobj.captureFrameRate, ~, OTobj.exportFrameRate,...
        ~, OTobj.startTime, ~, ~, ~, OTobj.totalFrames, ~, OTobj.rotationType, ...
        ~, OTobj.lengthUnits, ~, ~] = textread(filename, '%s %f %s %s %s %f %s %f %s %s %s %d %s %d %s %s %s %s %s %s', ...
        1, 'delimiter', ',');
    
    % Import all data
    try
    tmp = importdata(filename, ',', 7); 
    catch
          tmp = importdata(filename, ',', 8); 
        disp('Assuming the data are in quaternions...');
    end
    % Change header rows so formatted neatly
    if ~isfield(tmp, 'colheaders')
        tmp.colheaders = tmp.textdata(end,:);
    end
    for i = 1:(size(tmp.textdata,1)-1)
        a = textscan(tmp.textdata{i,1}, '%s',...
            'delimiter', ',');
        tmp.textdata(i,:) = cell(1, length(tmp.textdata(end,:)));
        tmp.textdata(i,1:length(a{1,1})) = transpose(a{1,1});
    end
%     % Select only columns containing data
%     i = find(any(~isnan(tmp.data), 1));
%     tmp.data = tmp.data(:,i);
%     tmp.dataCols = tmp.textdata(:,i);
%    
    % Structure data
    % Rigid bodies
    RBcols = find(strcmp(tmp.textdata(2,:), 'Rigid Body'));
    RBcols = transpose(reshape(RBcols, 8, []));
    noRBs = size(RBcols, 1);
    for i = 1:noRBs
        try
            OTobj.rigidbodies(i).data = tmp.data(:, RBcols(i,:));
        catch
            disp('');
        end
        OTobj.rigidbodies(i).colheaders = strcat(tmp.textdata(5, RBcols(i,:)), ...
            tmp.colheaders(:, RBcols(i,:)));
        OTobj.rigidbodies(i).name = tmp.textdata(3, RBcols(i,1));
    end
    
    % Markers
    % Rigid body markers
    RBmarkercols = find(strcmp(tmp.textdata(2,:), 'Rigid Body Marker'));
    RBmarkercols = transpose(reshape(RBmarkercols, 4, []));
    noRBmarkers = size(RBmarkercols, 1);
    for i = 1:noRBmarkers
        OTobj.markers.rigidbodymarkers(i).data = tmp.data(:, RBmarkercols(i,:));
        OTobj.markers.rigidbodymarkers(i).colheaders = strcat(tmp.textdata(5, RBmarkercols(i,:)), ...
            tmp.colheaders(:, RBmarkercols(i,:)));
        OTobj.markers.rigidbodymarkers(i).name = strrep(tmp.textdata{3,RBmarkercols(i,1)},'"','');
    end
    
    % Labeled markers
    Lmarkercols = find(strcmp(tmp.textdata(2,:), 'Marker'));
    Lmarkercols = transpose(reshape(Lmarkercols, 3, []));
    noLmarkers = size(Lmarkercols, 1);
    for i = 1:noLmarkers
       OTobj.markers.labeledmarkers(i).data = tmp.data(:, Lmarkercols(i,:));
       OTobj.markers.labeledmarkers(i).colheaders = strcat(tmp.textdata(5, Lmarkercols(i,:)), ...
            tmp.colheaders(:, Lmarkercols(i,:)));
       OTobj.markers.labeledmarkers(i).name = strrep(tmp.textdata{3,Lmarkercols(i,1)},'"','');
    end 
    
    % Unlabeled markers
    ULmarkercols = find(strcmp(tmp.textdata(2,:), 'Unlabeled Marker'));
    ULmarkercols = transpose(reshape(ULmarkercols, 3, []));
    noULmarkers = size(ULmarkercols, 1);
    for i = 1:noULmarkers
       OTobj.markers.unlabeledmarkers(i).data = tmp.data(:, ULmarkercols(i,:));
       OTobj.markers.unlabeledmarkers(i).colheaders = strcat(tmp.textdata(5, ULmarkercols(i,:)), ...
            tmp.colheaders(:, ULmarkercols(i,:)));
       OTobj.markers.unlabeledmarkers(i).name = strrep(tmp.textdata{3,ULmarkercols(i,1)},'"','');
    end 
    
    % Time
    OTobj.time = tmp.data(:,2);
  
elseif strcmp(FullFile{2}, '1.23')
    % Get setup info
    [~, ~, ~, OTobj.takeName, ~,~,~, OTobj.captureFrameRate, ~, OTobj.exportFrameRate,...
        ~, OTobj.startTime, ~, ~, ~, OTobj.totalFrames, ~, OTobj.rotationType, ...
        ~, OTobj.lengthUnits, ~, ~] = textread(filename, '%s %f %s %s %s %s %s %f %s %f %s %s %s %d %s %d %s %s %s %s %s %s', ...
        1, 'delimiter', ',');
    
    % Import all data
    tmp = importdata(filename, ',', 7); 
    % Change header rows so formatted neatly
    if ~isfield(tmp, 'colheaders')
        tmp.colheaders = tmp.textdata(end,:);
    end
    for i = 1:(size(tmp.textdata,1)-1)
        a = textscan(tmp.textdata{i,1}, '%s',...
            'delimiter', ',');
        tmp.textdata(i,:) = cell(1, length(tmp.textdata(end,:)));
        tmp.textdata(i,1:length(a{1,1})) = transpose(a{1,1});
    end
%     % Select only columns containing data
%     i = find(any(~isnan(tmp.data), 1));
%     tmp.data = tmp.data(:,i);
%     tmp.dataCols = tmp.textdata(:,i);
%    
    % Structure data
    % Rigid bodies
    RBcols = find(strcmp(tmp.textdata(2,:), 'Rigid Body'));
    try
    RBcols = transpose(reshape(RBcols, 7, []));
    catch
            RBcols = transpose(reshape(RBcols, 8, []));
    end

    noRBs = size(RBcols, 1);
    for i = 1:noRBs
        try
            OTobj.rigidbodies(i).data = tmp.data(:, RBcols(i,:));
        catch
            disp('');
        end
        OTobj.rigidbodies(i).colheaders = strcat(tmp.textdata(5, RBcols(i,:)), ...
            tmp.colheaders(:, RBcols(i,:)));
        OTobj.rigidbodies(i).name = tmp.textdata(3, RBcols(i,1));
    end
    
    % Markers
    % Rigid body markers
    RBmarkercols = find(strcmp(tmp.textdata(2,:), 'Rigid Body Marker'));
    RBmarkercols = transpose(reshape(RBmarkercols, 4, []));
    noRBmarkers = size(RBmarkercols, 1);
    for i = 1:noRBmarkers
        OTobj.markers.rigidbodymarkers(i).data = tmp.data(:, RBmarkercols(i,:));
        OTobj.markers.rigidbodymarkers(i).colheaders = strcat(tmp.textdata(5, RBmarkercols(i,:)), ...
            tmp.colheaders(:, RBmarkercols(i,:)));
        OTobj.markers.rigidbodymarkers(i).name = strrep(tmp.textdata{3,RBmarkercols(i,1)},'"','');
    end
    
    % Labeled markers
    Lmarkercols = find(strcmp(tmp.textdata(2,:), 'Marker'));
    Lmarkercols = transpose(reshape(Lmarkercols, 3, []));
    noLmarkers = size(Lmarkercols, 1);
    for i = 1:noLmarkers
       OTobj.markers.labeledmarkers(i).data = tmp.data(:, Lmarkercols(i,:));
       OTobj.markers.labeledmarkers(i).colheaders = strcat(tmp.textdata(5, Lmarkercols(i,:)), ...
            tmp.colheaders(:, Lmarkercols(i,:)));
       OTobj.markers.labeledmarkers(i).name = strrep(tmp.textdata{3,Lmarkercols(i,1)},'"','');
    end 
    
    % Unlabeled markers
    ULmarkercols = find(strcmp(tmp.textdata(2,:), 'Unlabeled Marker'));
    ULmarkercols = transpose(reshape(ULmarkercols, 3, []));
    noULmarkers = size(ULmarkercols, 1);
    for i = 1:noULmarkers
       OTobj.markers.unlabeledmarkers(i).data = tmp.data(:, ULmarkercols(i,:));
       OTobj.markers.unlabeledmarkers(i).colheaders = strcat(tmp.textdata(5, ULmarkercols(i,:)), ...
            tmp.colheaders(:, ULmarkercols(i,:)));
       OTobj.markers.unlabeledmarkers(i).name = strrep(tmp.textdata{3,ULmarkercols(i,1)},'"','');
    end 
    
    % Time
    OTobj.time = tmp.data(:,2);
else
    error('Unknown file format.');
end
disp('');
