clear;clc;
s = serial('COM4','BaudRate',57600,'Terminator','CR');
fopen(s);
%讀取資料 1000筆 => 10s
data_length = 1000;
data=zeros(data_length,3);
fscanf(s);
fscanf(s);
fscanf(s);
fscanf(s);
for i=1:data_length
    str = fscanf(s);
    token = split(str);
    data(i,1) = str2double(token(2));
    data(i,2) = str2double(token(3));
    data(i,3) = str2double(token(4));
end

%所需範圍取值
dt = 0.01;
data_h = ;
data_t = ;
data_length = data_t-data_h;
%輸入電壓
v = ;

U = zeros(data_length,1) + v;
thetaA = data(data_h:data_t,1);
thetaB = data(data_h:data_t,2);
sigma = data(data_h:data_t,3);

dthetaA = zeros(data_length);
dthetaB = zeros(data_length);
dsigma = zeros(data_length);
for i=1 : (size(sigma)-1)
    dthetaA(i) = (thetaA(i+1)-thetaA(i))/dt;
    dthetaB(i) = (thetaB(i+1)-thetaB(i))/dt;
    dsigma(i) = (sigma(i+1)-sigma(i))/dt;
end
dthetaA(data_length) = (thetaA(data_length)-thetaA(data_length-1))/dt;
dthetaB(data_length) = (thetaB(data_length)-thetaB(data_length-1))/dt;
dsigma(data_length) = (sigma(data_length)-sigma(data_length-1))/dt;


ddthetaA = zeros(data_length);
ddthetaB = zeros(data_length);
ddsigma = zeros(data_length);
for i=1 : (size(sigma)-1)
    ddthetaA(i) = (dthetaA(i+1)-dthetaA(i))/dt;
    ddthetaB(i) = (dthetaB(i+1)-dthetaB(i))/dt;
    ddsigma(i) = (dsigma(i+1)-dsigma(i))/dt;
end
ddthetaA(data_length) = (dthetaA(data_length)-dthetaA(data_length-1))/dt;
ddthetaB(data_length) = (dthetaB(data_length)-dthetaB(data_length-1))/dt;
ddsigma(data_length) = (dsigma(data_length)-dsigma(data_length-1))/dt;


E = ddsigma;
G = [-ddthetaA, -2*cos(sigma)*ddsigma+sin(2*sigma)*sec(sigma)*(dsigma^2), -dthetaA, dsigma, U];
    
objs = instrfind;
fclose(objs);