# Optitrack at the Wellcome Centre for Human Neuroimaging OP-MEG lab:
Scripts and SOPs for collecting, processing and plotting the Optitrack motion capture data


# Things this Repository can do:


### Load in optitrack data

This function can deal with rotation data in both quaternions and euler angles

```matlab
 opti_data = csv2mat_sm(motive_data);
```
### Convert optitrack data from mm to cm

```matlab
opti_data = optitrack_to_cm(opti_data);
```
### Synchronise and upsample the opti-track data to match the OP-MEG data

```matlab
%% Sync up opti-track and OP-MEG data
[MovementDataOut, OPMdataOut] = syncOptitrackAndOPMdata(opti_data,...
	rawData,'TriggerChannelName','FluxZ-A');
```

### Plot the data

Plot the continuous rotations, translations and mean marker error

```matlab
% Plot the rotations
plot_motive_rotation(opti_data,'euler')
    
% Plot the translations
plot_motive_translation(opti_data,'euler')

% Plot mean marker error (Column 7 for euler data; 8 for quaternions)
figure;
plot(opti_data.time,opti_data.rigidbodies.data(:,7),'LineWidth',2);
ylabel('Mean Marker Error');xlabel('Time (s)');
```