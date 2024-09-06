function [dataout] = optitrack_convert_units(opti_data,newUnits)
%function [dataout] = optitrack_convert_units(opti_data,newUnits)
%   Convert optitrack length units
%
% INPUT:
%   - opti_data: optitrack data, output from csv2mat_sm
%   - newUnits: new length units. Either 'mm', 'cm' or 'm'
%
% OUTPUT:
%   - dataout: optitrack data with new length units
%
% FUNCTION DEPENDENCIES:
%   None
%
% AUTHOR
%   Stephanie Mellor, 28/11/22
%

dataout = opti_data;

% Get current units
if isfield('opti_data', 'cfg')
    data_format_type = 'new';
else
    data_format_type = 'old';
end

cur_unit = opti_data.lengthUnits{1};
dataout.lengthUnits{1} = newUnits;

% Find multiplication factor
switch cur_unit
    case {'Meters', 'm'}
        if strcmp(newUnits, 'm')
            warning('OptiTrack units unchanged; new units input were same as existing');
            return;
        elseif strcmp(newUnits, 'cm')
            multfact = 100;
        elseif strcmp(newUnits, 'mm')
            multfact = 1e3;
        end
    case {'Centimeters', 'cm'}
        if strcmp(newUnits, 'm')
            multfact = 1e-2;
        elseif strcmp(newUnits, 'cm')
            warning('OptiTrack units unchanged; new units input were same as existing');
            return;
        elseif strcmp(newUnits, 'mm')
            multfact = 10;
        end
    case {'Millimeters', 'mm'}
        if strcmp(newUnits, 'm')
            multfact = 1e-3;
        elseif strcmp(newUnits, 'cm')
            multfact = 1e-1;
        elseif strcmp(newUnits, 'mm')
            warning('OptiTrack units unchanged; new units input were same as existing');
            return;
        end
    otherwise
        error('Current units in opti_data not recognised');
end

% Update data
% Change rigidbodies.data
for i = 1:length(opti_data.rigidbodies)
    indx = contains(opti_data.rigidbodies(i).colheaders,'Position');
    dataout.rigidbodies(i).data(:,indx) = multfact*dataout.rigidbodies(i).data(:,indx);
end

% Change markers.rigidbodymarkers.data
for i = 1:length(opti_data.markers.rigidbodymarkers)
    indx = contains(opti_data.markers.rigidbodymarkers(i).colheaders,'Position');
    dataout.markers.rigidbodymarkers(i).data(:,indx) = ...
        multfact*opti_data.markers.rigidbodymarkers(i).data(:,indx);
end
    
% Change markers.labeledmarkers.data
for i = 1:length(opti_data.markers.labeledmarkers)
    indx = contains(opti_data.markers.labeledmarkers(i).colheaders,...
        'Position');
    dataout.markers.labeledmarkers(i).data(:,indx) = ...
        multfact*opti_data.markers.labeledmarkers(i).data(:,indx);
end
end