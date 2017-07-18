function oxts = loadInertialData()
% reads GPS/IMU data from files to memory. please modify the data directory

k = 1;
oxts = [];
for i=1:5000000
    try
        file_name = ['G:\SelfRecordData\Data\Record8\GPS\' num2str(i,'%06d') '.txt'];
        oxts{k} = dlmread(file_name);
    catch e
        break;
    end
    k=k+1;
end

