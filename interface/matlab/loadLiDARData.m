function [velo] = loadLiDARData (frame)
% please modify the data directory
% Input arguments:
% frame index
dir = ['G:/SelfRecordData/Data/Record7/Velo' '/%06d.bin']; 
fid = fopen(sprintf(dir,frame),'rb');
velo = fread(fid,[5 inf],'single')';
fclose(fid);

end
