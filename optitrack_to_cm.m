function [dataout] = optitrack_to_cm(datain)

if strcmp(datain.lengthUnits{1},'Millimeters')
    disp('Converting position data from mm to cm');
    dataout = datain;
    dataout.lengthUnits{1} = 'cm';
    
    % Change rigidbodies.data
    indx = contains(datain.rigidbodies.colheaders,'Position');
    dataout.rigidbodies.data(:,indx) = datain.rigidbodies.data(:,indx)/10;
    
    % Change markers.rigidbodymarkers.data
    try
        for i = 1:length(datain.markers.rigidbodymarkers)
            
            indx = contains(datain.markers.rigidbodymarkers(i).colheaders,...
                'Position');
            dataout.markers.rigidbodymarkers(i).data(:,indx) = ...
                datain.markers.rigidbodymarkers(i).data(:,indx)/10;
        end
    catch
        disp('Could not process markers.rigidbodymarkers');
    end
    
    %Change markers.labeledmarkers.data
    
    try
        for i = 1:length(datain.markers.labeledmarkers)
            
            indx = contains(datain.markers.labeledmarkers(i).colheaders,...
                'Position');
            dataout.markers.labeledmarkers(i).data(:,indx) = ...
                datain.markers.labeledmarkers(i).data(:,indx)/10;
        end
    catch
        disp('Could not process markers.labeledmarkers');
    end
else
    disp('Could not process')
end
