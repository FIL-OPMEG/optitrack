function [ MagPos, MagOri, R, T ] = getMagPosOriOverTime( MovementData, MarkerPosMRI, D, RigidBodyName )
%function [ MagPos, MagOri ] = getMagPosOriOverTime( MovementData, MarkerPosMRI, D )
%   Get the magnetometer positions and orientations over the course of an
%   experiment. Uses the Kabsch method to match the position of markers in
%   native MRI space to their position in Optitrack space at each time
%   point and then applies the same transformation to the magnetometer
%   positions and orientations given in an spm meeg object. 
%
% INPUT:
%   - MovementData: optitrack data from csv2mat_sm. 
%   - MarkerPosMRI: marker positions in native MRI space. Markers should be
%       ordered to match their numbering in MovementData. 
%   - D: spm meeg object, containing a sensors variable, from which the
%       magnetometer positions and orientations in native MRI space will be
%       taken.
%   - RigidBodyName: name of rigid body of interest. Leave blank ([]) if 
%       using "old" (csv2mat_sm) style software to read in optitrack data
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

if nargin < 4
    RigidBodyName = [];
end

% Get magnetometer position in MRI coords
magP_MRI = D.sensors('MEG').chanpos';
magO_MRI = D.sensors('MEG').chanori';

% Find rotation and translation between MRI and room spaces using Kabsch
% method
markerPos_Camera = [];
if isempty(RigidBodyName)
    for i = 1:length(MovementData.markers.rigidbodymarkers)
        markerPos_Camera = cat(3, markerPos_Camera, MovementData.markers.rigidbodymarkers(i).data(:,1:3)');
    end
else
    for i = 1:((size(MovementData.(RigidBodyName).RigidBodyMarker, 2)-2)/4)
        markerPos_Camera = cat(3, markerPos_Camera, MovementData.(RigidBodyName).RigidBodyMarker{:,4*i-1:4*i+1}');
    end
end

A_trans = MarkerPosMRI;
CA_trans = mean(A_trans, 1);
R = zeros(3, 3, size(markerPos_Camera, 2));
T = zeros(3, size(markerPos_Camera,2));
for j = 1:size(markerPos_Camera,2)
    B_trans = transpose(squeeze(markerPos_Camera(:,j,:)));
    CB_trans = mean(B_trans, 1);
    [U,~,V] = svd((A_trans - CA_trans)'*(B_trans - CB_trans));
    d = det(V*U');
    R(:,:,j) = V*[1 0 0; 0 1 0; 0 0 sign(d)]*U';
    T(:,j) = CB_trans' - R(:,:,j)*CA_trans';
end

% Get in camera space
MagPos = zeros(3, size(D,2), size(D.sensors('MEG').chanpos,1));
MagOri = zeros(3, size(D,2), size(D.sensors('MEG').chanpos,1));

for j = 1:size(MagPos,2)
    MagPos(:,j,:) = R(:,:,j)*magP_MRI + T(:,j);
    MagOri(:,j,:) = R(:,:,j)*magO_MRI;
end


end

