function [ MarkerPosMRI ] = getMarkerPosInMRIcoords( scannerCastTableOfInfo, markerScannerCastSlots, markerHeights, MovementData, radAxis )
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
%   - radAxis: radial axis of the scanner-cast. String s.t. if OPM Z axis
%       is radial, set to 'RAD', else if Y axis is radial to the head, set
%       to 'TAN'. Default 'RAD'
%
% OUTPUT:
%   - MarkerPosMRI: Native space coordinates of markers. 
%       Nmarkers x 3 (column = spacial dimension).
%
% FUNCTION DEPENDENCIES:
%    None
%
% AUTHOR INFO:
% Stephanie Mellor, stephanie.mellor.17@ucl.ac.uk
% 26/1/21
%

% Set default radial axis
if nargin < 5
    radAxis = 'RAD';
end

% Import table of info
scannerCast = readtable(scannerCastTableOfInfo);

% Get marker positions in native space
[~, idx, ~] = intersect(scannerCast.Headcast, markerScannerCastSlots, 'stable');
markersP = [scannerCast.Px(idx), scannerCast.Py(idx), scannerCast.Pz(idx)];

% Add the height of the marker
for i = 1:length(markerHeights)
    if strcmp(radAxis, 'RAD')
        markersP(i,:) = markersP(i,:) +...
            markerHeights(i)*[scannerCast.Ox_RAD(idx(i)), scannerCast.Oy_RAD(idx(i)), scannerCast.Oz_RAD(idx(i))];
    elseif strcmp(radAxis, 'TAN')
        markersP(i,:) = markersP(i,:) +...
            markerHeights(i)*[scannerCast.Ox_TAN(idx(i)), scannerCast.Oy_TAN(idx(i)), scannerCast.Oz_TAN(idx(i))];
    end
end

% Then use ICP to make sure that marker 1 on the scanner-cast is marker 1
% in the Optitrack recording 

% Get first non-NaN position of all markers in movement data
nMarkers = length(MovementData.markers.labeledmarkers);
pOptiTrackInit = NaN(3, nMarkers);
counter = 1;
while any(isnan(pOptiTrackInit(:)))
    for i = 1:nMarkers
        pOptiTrackInit(:,i) = MovementData.markers.labeledmarkers(i).data(counter,:)';
    end
    counter = counter + 1;
end

% Use ICP to register markersP (from headcast) and pOptiTrackInit (from
% OptiTrack) to get the positions from the headcast and the numbers from
% the OptiTrack
moving = pOptiTrackInit;
fixed = markersP';
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
        fprintf('Reached maximum number of iterations\n');
        error('MRI and optitrack coordinates could not be matched.\nEnsure the correct scanner-cast slots and heights have been used.');
    end
end
clear idxs

% Label markersP by nearest OptiTrack marker after transformation
moving = TR*moving + TT;
markersOrder = zeros(nMarkers,1);
for i = 1:nMarkers
    [~, markersOrder(i)] = min(sqrt(sum((markersP - moving(:,i)').^2, 2)));
end
MarkerPosMRI = markersP(markersOrder, :);

end

