function [dataout] = zero_optitrack_data(datain)
first_point = repmat(datain(1,:),size(datain,1),1);
dataout = datain-first_point; clear first_point
end
