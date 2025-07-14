function [grad] = extractGradSelectedSensors(D)
% Helper function to extract the grad structure for only the selected
% sensors from a trimmed SPM D object (i.e. a D object where the number of 
% channels was reduced using the function spm_eeg_epoch(). The use case of
% this function, is to derive a filtered grad structure, for which the
% position and orientation can flexibly updated for every point in time,
% using movment data from an Optitrack recording. 
% 
% INPUT:
%   - OPMdata options:
%       - D: SPM meeg object containing OPM data. If this option is used, SPM12
%           must be installed on the PC and in the matlab file path.
%
% OUTPUT:
%   - grad: grad structure for the selected sensors, in the same
%   order as in the SPM D object.
%
% % Example use:
% % (see individual functions for cfg specifications)
%
% [sensorLevelRbTimeseries] = getChannelLevelRigidBodyTimeseries(cfg);
% cD = spm_eeg_crop(cfg);
% grad = extractGradSelectedSensors(cD);
% grad = updateSensorPositionsFrame(grad, sensorLevelRbTimeseries, sampleIdx);
%
% Author:	Sascha Woelk (s.wolk.16@ucl.ac.uk)
% MIT License

% Find the labels and indices of selected channels
D_chanlabels = D.chanlabels(D.indchantype('meg'));
[sensorBool, sensorIdx] = ismember(D_chanlabels, D.sensors('meg').label);
sensorIdx = sensorIdx(sensorBool); % Keep only matching entries

% Exract the sensor info for the selected channels
chanori_select = D.sensors('meg').chanori(sensorIdx,:);
chanpos_select = D.sensors('meg').chanpos(sensorIdx,:);
chantype_select = D.sensors('meg').chantype(sensorIdx,:);
chanunit_select = D.sensors('meg').chanunit(sensorIdx,:);
coilori_select = D.sensors('meg').coilori(sensorIdx,:);
coilpos_select = D.sensors('meg').coilpos(sensorIdx,:);
label_select = D.sensors('meg').label(sensorIdx,:);
tra_select = D.sensors('meg').tra(1:size(label_select,1),1:size(label_select,1));

% Set the new sensor field values based on the selected channels
grad = sensors(D, 'meg');
grad.chanori = chanori_select;
grad.chanpos = chanpos_select;
grad.chantype = chantype_select;
grad.chanunit = chanunit_select;
grad.coilori = coilori_select;
grad.coilpos = coilpos_select;
grad.label = label_select;
grad.tra = tra_select;