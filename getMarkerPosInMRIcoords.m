function [ MarkerPosMRI ] = getMarkerPosInMRIcoords( scannerCastTableOfInfo, ...
    markerScannerCastSlots, markerHeights, MovementData, RigidBodyName, radAxis )
%function [ MarkerPosMRI ] = getMarkerPosInMRIcoords( scannerCastTableOfInfo, markerScannerCastSlots, markerHeights, MovementData, radAxis )
%   Get the position of each optitrack marker in the native MRI space of
%   a participant's scanner-cast. Needed to get the magnetometer positions
%   and orientations over time.
%
% INPUT:
%   - scannerCastTableOfInfo: A file containing the native space
%       coordinates of each of the scanner-cast slots. Currently only set
%       up for csv files with column headers including [Headcast, Px, Py,
%       Pz, Ox_RAD, Oy_RAD, Oz_RAD, Ox_TAN, Oy_TAN, Oz_TAN].
%   - markerScannerCastSlots: The numbers of the slots the markers were
%       placed in/on. Double array with as many elements as markers.
%   - markerHeights: Height of markers above scanner-cast slot center.
%       Double array with as many elements as markers.
%   - MovementData: The optitrack data to match the markers to. Currently
%       only set up for data from csv2mat_sm. 
%   - RigidBodyName: Name of rigid body to match with MRI. Should be left
%       empty ([]) if "old" (csv2mat_sm) function was used to read in OptiTrack
%       data
%   - radAxis: radial axis of the scanner-cast. String s.t. if OPM Z axis
%       is radial, set to 'Z', else if Y axis is radial to the head, set
%       to 'Y'. Default 'Y'
%
% OUTPUT:
%   - MarkerPosMRI: Native space coordinates of markers. 
%       Nmarkers x 3 (column = spacial dimension).
%
% FUNCTION DEPENDENCIES:
%    icp
%
% AUTHOR INFO:
% Stephanie Mellor, stephanie.mellor.17@ucl.ac.uk
% 20/4/21
%

% Set default radial axis
if nargin < 6
    radAxis = 'Y';
    if nargin < 5
        RigidBodyName = [];
    end
end

% Import table of info
scannerCast = readtable(scannerCastTableOfInfo);

% Get marker positions in native space
try
    [~, idx, ~] = intersect(scannerCast.Headcast, markerScannerCastSlots, 'stable');
catch
    [~, idx, ~] = intersect(scannerCast.SlotNo, markerScannerCastSlots, 'stable');
end
if ismember('Px', scannerCast.Properties.VariableNames)
    markersP = [scannerCast.Px(idx), scannerCast.Py(idx), scannerCast.Pz(idx)];
elseif ismember('Position_x', scannerCast.Properties.VariableNames)
    markersP = [scannerCast.Position_x(idx), scannerCast.Position_y(idx), scannerCast.Position_z(idx)];
elseif ismember('G2_P_X', scannerCast.Properties.VariableNames)
    markersP = [scannerCast.G2_P_X(idx), scannerCast.G2_P_Y(idx), scannerCast.G2_P_Z(idx)];
    tmpXOri = [scannerCast.G2_Ox_X(idx), scannerCast.G2_Oy_X(idx), scannerCast.G2_Oz_X(idx)];
    tmpZOri = [scannerCast.G2_Ox_Z(idx), scannerCast.G2_Oy_Z(idx), scannerCast.G2_Oz_Z(idx)];
    markersP = markersP + 1.9.*tmpXOri - 5.7*tmpZOri;
end

% Add the height of the marker
rad_ax = zeros(length(markerHeights), 3);
for i = 1:length(markerHeights)
    if ismember('Ox_RAD', scannerCast.Properties.VariableNames)
        if strcmp(radAxis, 'Z')
            rad_ax(i,:) = [scannerCast.Ox_RAD(idx(i)), scannerCast.Oy_RAD(idx(i)), scannerCast.Oz_RAD(idx(i))];
        elseif strcmp(radAxis, 'Y')
            rad_ax(i,:) = [scannerCast.Ox_TAN(idx(i)), scannerCast.Oy_TAN(idx(i)), scannerCast.Oz_TAN(idx(i))];
        end
    elseif ismember('Ox_Z', scannerCast.Properties.VariableNames)
        if strcmp(radAxis, 'Z')
            rad_ax(i,:) = [scannerCast.Ox_Z(idx(i)), scannerCast.Oy_Z(idx(i)), scannerCast.Oz_Z(idx(i))];
        elseif strcmp(radAxis, 'Y')
            rad_ax(i,:) = [scannerCast.Ox_Y(idx(i)), scannerCast.Oy_Y(idx(i)), scannerCast.Oz_Y(idx(i))];
        end
    elseif ismember('Z_x', scannerCast.Properties.VariableNames)
        if strcmp(radAxis, 'Z')
            rad_ax(i,:) = [scannerCast.Z_x(idx(i)), scannerCast.Z_y(idx(i)), scannerCast.Z_z(idx(i))];
        elseif strcmp(radAxis, 'Y')
            rad_ax(i,:) = [scannerCast.Y_x(idx(i)), scannerCast.Y_y(idx(i)), scannerCast.Y_z(idx(i))];
        end
    elseif ismember('G2_Ox_X', scannerCast.Properties.VariableNames)
        if strcmp(radAxis, 'Z')
            rad_ax(i,:) = [scannerCast.G2_Ox_Z(idx(i)), scannerCast.G2_Oy_Z(idx(i)), scannerCast.G2_Oz_Z(idx(i))];
        elseif strcmp(radAxis, 'Y')
            rad_ax(i,:) = [scannerCast.G2_Ox_Y(idx(i)), scannerCast.G2_Oy_Y(idx(i)), scannerCast.G2_Oz_Y(idx(i))];
        end
    else
        error('Scanner-cast table_of_info format not recognised.')
    end
end

% Then use ICP to make sure that marker 1 on the scanner-cast is marker 1
% in the Optitrack recording 

% Get first non-NaN position of all markers in movement data
if isempty(RigidBodyName)
    nMarkers = length(MovementData.markers.labeledmarkers);
else
    nMarkers = (size(MovementData.(RigidBodyName).RigidBodyMarker, 2) - 2)/4;
end
pOptiTrackInit = NaN(3, nMarkers);
counter = 1;
while any(isnan(pOptiTrackInit(:)))
    for i = 1:nMarkers
        if isempty(RigidBodyName)
            pOptiTrackInit(:,i) = MovementData.markers.rigidbodymarkers(i).data(counter,1:3)';
        else
            pOptiTrackInit(:,i) = MovementData.(RigidBodyName).RigidBodyMarker{counter,4*i-1:4*i+1}';
        end
    end
    counter = counter + 1;
end

% Multiply to get in mm
if isempty(RigidBodyName)
    lu = MovementData.lengthUnits{:};
else
    lu = MovementData.cfg.LengthUnits;
end
if strcmp(lu, 'Centimeters')
    pOptiTrackInit = pOptiTrackInit*10;
elseif strcmp(lu, 'Meters')
    pOptiTrackInit = pOptiTrackInit*1000;
end

% Use ICP to register markersP (from headcast) and pOptiTrackInit (from
% OptiTrack) to get the positions from the headcast and the numbers from
% the OptiTrack
moving = pOptiTrackInit;
er_neg_or_pos = [0,0];
reached_max_iter_check = 0;

for neg_or_pos_ax = 1:2 % check to mean the user doesn't need to know 
    % whether the height of the marker should be added or subtracted along the radial helmet axis

    if neg_or_pos_ax == 1
        markersP_height = markersP + repmat(markerHeights', 1, 3).*rad_ax;
    else
        markersP_height = markersP - repmat(markerHeights', 1, 3).*rad_ax;
    end
    fixed = markersP_height';
    [TR, TT, ER] = icp(fixed, moving, 10);
    iter = 1;
    % Reinitialise if stuck in local minimum
    while ER(end) > 10
        moving = TR * moving + TT;
        
        % Random rotation
        randRot = randn(1, 3);
        R = [1 0 0; 0 cos(randRot(1)) -sin(randRot(1)); 0 sin(randRot(1)) cos(randRot(1))]*...
            [cos(randRot(2)) 0 sin(randRot(2)); 0 1 0; -sin(randRot(2)) 0 cos(randRot(2))]*...
            [cos(randRot(3)) -sin(randRot(3)) 0; sin(randRot(3)) cos(randRot(3)) 0; 0 0 1];
        clear randRot
        T = 0.1*randn(3,1);
        
        % New moving
        moving = R*moving + T;
        clear R T
        
        % Try again
        [TR, TT, ER] = icp(fixed, moving, 100);
        
        iter = iter + 1;
        if iter > 1000
            if neg_or_pos_ax == 1
                reached_max_iter_check = 1;
            end
            if neg_or_pos_ax == 2 && reached_max_iter_check
                fprintf('Reached maximum number of iterations\n');
            end
            break;
            %error('MRI and optitrack coordinates could not be matched. Ensure the correct scanner-cast slots and heights have been used.');
        end
    end
    TR_tmp{neg_or_pos_ax} = TR;
    TT_tmp{neg_or_pos_ax} = TT;
    er_neg_or_pos(neg_or_pos_ax) = ER(end);
    clear idxs
end

[~, ax_ind] = min(er_neg_or_pos);
TR = TR_tmp{ax_ind};
TT = TT_tmp{ax_ind};

% Label markersP by nearest OptiTrack marker after transformation
moving = TR*moving + TT;
markersOrder = zeros(nMarkers,1);
for i = 1:nMarkers
    [~, markersOrder(i)] = min(sqrt(sum((markersP - moving(:,i)').^2, 2)));
end
MarkerPosMRI = markersP(markersOrder, :);

end

