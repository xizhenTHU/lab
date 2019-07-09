%轨迹分割与分析
clear;close all
filename = 'E:\SoftwareTest\PythonTest\mmw\mmw_time\time_2019_05_31_16_21_47.txt';
delimiter = ' ';
formatSpec = '%*s%*s%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string', 'EmptyValue', NaN,  'ReturnOnError', false);
fclose(fileID);
data = [dataArray{1:end-1}];
clearvars filename delimiter formatSpec fileID dataArray ans;
%去除重复数据
[~,ia,~] = unique(data(:,1:2),'rows','stable');
data=data(ia,:);
%去除相近数据
dist_lla=sqrt(diff(data(:,1)).^2+diff(data(:,2)).^2)*1.015271759128623e+05+0.006120193348871;
while sum(dist_lla<0.15+0.05)>0
%     data(logical([1;dist_lla<0.15+0.05]),:)=[];
    data(logical([0;dist_lla<0.15+0.05]),:)=[];
    dist_lla=sqrt(diff(data(:,1)).^2+diff(data(:,2)).^2)*1.015271759128623e+05+0.006120193348871;
end
% lng=data(:,1);lat=data(:,2);yaw=data(:,3);
% lng=data(100:200,1);lat=data(100:200,2);yaw=data(100:200,3);
num_cut=110:130;num_cut=110:120;lng=data(num_cut,1);lat=data(num_cut,2);yaw=data(num_cut,3);
figure;plot(lat,lng,'.-');axis equal

lla=[lat,lng,zeros(size(lat))];p = lla2ecef(lla, 'WGS84');
[xEast,yNorth,~] = ecef2enu(p(:,1),p(:,2),p(:,3),lat(1),lng(1),0,wgs84Ellipsoid);
figure;plot(xEast,yNorth,'.-');axis equal;xlabel('East');ylabel('North');

%%
%插值
x_out=linspace(min(xEast),max(xEast),5*length(xEast)).';
y_out = interp1(xEast,yNorth,x_out,'spline');
figure;plot(x_out,y_out,'.-');
hold on;plot(xEast,yNorth,'.-');axis equal;xlabel('East');ylabel('North');

% x_out=linspace(min(lat),max(lat),10*length(lat)).';
% y_out = interp1(lat,lng,x_out,'spline');
% figure;plot(x_out,y_out,'.-');axis equal;xlabel('East');ylabel('North');

% diff_angle=atan2d(diff(x_out),diff(y_out));
diff_angle=atan2d(diff(xEast),diff(yNorth));
for ii=1:length(diff_angle)
	if diff_angle(ii)<0
        diff_angle(ii)=diff_angle(ii)+360;
	end
end    

figure;plot(xEast,yNorth,'.-');hold on;
quiver(xEast(2:end),yNorth(2:end),sind(diff_angle),cosd(diff_angle),0.5);axis equal;xlabel('East');ylabel('North');


figure;plot(1:100,[diff_angle,yaw(2:end,:)],'.-');
diff(diff_angle)<5


diff(yNorth)
diff(xEast)
diff(lat)
diff(lng)

