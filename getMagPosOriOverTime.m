function [ MagPos, MagOri ] = getMagPosOriOverTime( MovementData, MarkerPosMRI, D )
%function [ MagPos, MagOri ] = getMagPosOriOverTime( MovementData, MarkerPosMRI, D )
%   Get the magnetometer positions and orientations over the course of an
%   experiment. Uses the Kabsch method to match the position of markers in
%   native MRI space to their position in Optitrack space at each time
%   point and then applies the same transformation to the magnetometer
%   positions and orientations given in an spm meeg object. 
%
% INPUT:
%   - MovementData: optitrack data from csv2mat_sm. 
%   - MarkerPosMRI: marker positions in native MRI spice. Markers should be
%       ordered to match their numbering in MovementData. 
%   - D: spm meeg object, containing a sensors variable, from which the
%       magnetometer positions and orientations in native MRI space will be
%       taken.
%
% OUTPUT:
%   - MagPos: double array, 3 x Time x Nchans
%       Magnetometer position over time.
%   - MagOri: double array, 3 x Time x Nchans
%       Magnetometer orientation over time.
%
% FUNCTION DEPENDENCIES:
%   None
%
% AUTHOR INFO:
% Stephanie Mellor, stephanie.mellor.17@ucl.ac.uk
% 26/1/21
%

% Get magnetometer position in MRI coords
magP_MRI = D.sensors('MEG').chanpos';
magO_MRI = D.sensors('MEG').chanori';

% Find rotation and translation between MRI and room spaces using Kabsch
% method
markerPos_Camera = [];
for i = 1:length(MovementData.markers.labeledmarkers)
    markerPos_Camera = cat(3, markerPos_Camera, MovementData.markers.labeledmarkers(i).data');
end

A_trans = MarkerPosMRI;
CA_trans = mean(A_trans, 2);
R = zeros(3, 3, size(markerPos_Camera, 2));
T = zeros(3, size(markerPos_Camera,2));
for j = 1:size(markerPos_Camera,2)
    B_trans = squeeze(markerPos_Camera(:,j,:));
    CB_trans = mean(B_trans, 2);
    [U,~,V] = svd((A_trans - repmat(CA_trans, 1, 3))*(B_trans - repmat(CB_trans, 1, 3))');
    d = det(V*U');
    R(:,:,j) = V*[1 0 0; 0 1 0; 0 0 d]*U';
    T(:,j) = CB_trans - R(:,:,j)*CA_trans;
end

% Get in camera space
MagPos = zeros(3, size(D,2), size(D.sensors('MEG').chanpos,1));
MagOri = zeros(3, size(D,2), size(D.sensors('MEG').chanpos,1));

for j = 1:size(MagPos,2)
    MagPos(:,j,:) = R(:,:,j)*magP_MRI + T(:,j);
    MagOri(:,j,:) = R(:,:,j)*magO_MRI;
end


end

