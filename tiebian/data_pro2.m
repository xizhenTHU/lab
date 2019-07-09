clear
filename = 'E:\SoftwareTest\PythonTest\5d24664ec7dfb97ec0eb699c.txt';
delimiter = ',';
formatSpec = '%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);
fclose(fileID);
data = [dataArray{1:end-1}];
clearvars filename delimiter formatSpec fileID dataArray ans;

lat_act=data(:,1);lng_act=data(:,2);
lat_de=data(:,3);lng_de=data(:,4);

figure;plot(lat_act,lng_act,'.-')
figure;plot(lat_de,lng_de,'.-')
figure;plot([lat_de,lat_act],[lng_de,lng_act],'.-')

lla=[lat_act,lng_act,zeros(size(lat_act))];p = lla2ecef(lla, 'WGS84');
[xEast_act,yNorth_act,~] = ecef2enu(p(:,1),p(:,2),p(:,3),lat_act(1),lng_act(1),0,wgs84Ellipsoid);
lla=[lat_de,lng_de,zeros(size(lat_de))];p = lla2ecef(lla, 'WGS84');
[xEast_de,yNorth_de,~] = ecef2enu(p(:,1),p(:,2),p(:,3),lat_act(1),lng_act(1),0,wgs84Ellipsoid);

figure;plot([xEast_de,xEast_act],[yNorth_de,yNorth_act],'.-');axis equal

