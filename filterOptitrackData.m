function [ MovementDataOut ] = filterOptitrackData( MovementData, cutOffs, filterType )
%function [ MovementDataOut ] = filterOptitrackData( MovementData, cutOffs, filterType )
%   Function to filter Movement Data output from csv2mat_sm
%   Filtering is done using FieldTrip
%
% INPUT:
%   - MovementData: output from csv2mat_sm
%   - cutOffs (double vector): filter 3dB cut off point/s. One number if
%       using a low or high pass filter, two numbers for band pass or notch
%   - filterType (string): 'low', 'high', 'bandpass', or 'bandstop' for a
%       low pass, high pass, bandpass or bandstop/notch filter
%       respectively.
%  
% OUTPUT:
%   - MovementDataOut: filtered optitrack data
%
% FUNCTION DEPENDENCIES:
%   - ft_preproc_bandstopfilter
%   - ft_preproc_highpassfilter
%   - ft_preproc_lowpassfilter
%   - ft_preproc_bandpassfilter
%
% AUTHOR INFO:
% Stephanie Mellor, stephanie.mellor.17@ucl.ac.uk
% 26/1/21
%

% Create variable to save new movement data
MovementDataOut = MovementData;

% Find the sampling frequency
sf = mean(1./diff(MovementData.time));

% Filter rigid body data
for i = 1:length(MovementData.rigidbodies)
    switch filterType
        case 'low'
            MovementDataOut.rigidbodies(i).data = transpose(ft_preproc_lowpassfilter(...
                MovementData.rigidbodies(i).data', sf, cutOffs));
        case 'high'
            MovementDataOut.rigidbodies(i).data = transpose(ft_preproc_highpassfilter(...
                MovementData.rigidbodies(i).data', sf, cutOffs));
        case 'bandpass'
             MovementDataOut.rigidbodies(i).data = transpose(ft_preproc_bandpassfilter(...
                MovementData.rigidbodies(i).data', sf, cutOffs));
        case 'bandstop'
            MovementDataOut.rigidbodies(i).data = transpose(ft_preproc_bandstopfilter(...
                MovementData.rigidbodies(i).data', sf, cutOffs));
    end
end

% Filter rigid body marker data
for i = 1:size(MovementData.markers.rigidbodymarkers,2)
    switch filterType
        case 'low'
            MovementDataOut.markers.rigidbodymarkers(i).data = transpose(ft_preproc_lowpassfilter(...
                MovementData.markers.rigidbodymarkers(i).data', sf, cutOffs));
        case 'high'
            MovementDataOut.markers.rigidbodymarkers(i).data = transpose(ft_preproc_highpassfilter(...
                MovementData.markers.rigidbodymarkers(i).data', sf, cutOffs));
        case 'bandpass'
             MovementDataOut.markers.rigidbodymarkers(i).data = transpose(ft_preproc_bandpassfilter(...
                MovementData.markers.rigidbodymarkers(i).data', sf, cutOffs));
        case 'bandstop'
            MovementDataOut.markers.rigidbodymarkers(i).data = transpose(ft_preproc_bandstopfilter(...
                MovementData.markers.rigidbodymarkers(i).data', sf, cutOffs));
    end
end

% Filter other marker data
for i = 1:size(MovementData.markers.labeledmarkers,2)
    switch filterType
        case 'low'
            MovementDataOut.markers.labeledmarkers(i).data = transpose(ft_preproc_lowpassfilter(...
                MovementData.markers.labeledmarkers(i).data', sf, cutOffs));
        case 'high'
            MovementDataOut.markers.labeledmarkers(i).data = transpose(ft_preproc_highpassfilter(...
                MovementData.markers.labeledmarkers(i).data', sf, cutOffs));
        case 'bandpass'
             MovementDataOut.markers.labeledmarkers(i).data = transpose(ft_preproc_bandpassfilter(...
                MovementData.markers.labeledmarkers(i).data', sf, cutOffs));
        case 'bandstop'
            MovementDataOut.markers.labeledmarkers(i).data = transpose(ft_preproc_bandstopfilter(...
                MovementData.markers.labeledmarkers(i).data', sf, cutOffs));
    end
end

end

