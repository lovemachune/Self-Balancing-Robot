clear;clc;
s = serial('COM4','BaudRate',57600,'Terminator','CR');
fopen(s);
%讀取資料 1000筆 => 10s
data_length = 500;
data=zeros(data_length,3);
fscanf(s);
fscanf(s);
fscanf(s);
fscanf(s);
for i=1:data_length
    str = fscanf(s);
    token = split(str);
    while(size(token)<4)
        str = fscanf(s);
        token = split(str);
    end
    data(i,1) = str2double(token(2));
    data(i,2) = str2double(token(3));
    data(i,3) = str2double(token(4));
end
objs = instrfind;
fclose(objs);

%for i = 1:1000
%    data(i,1) = data(i,1)/180*pi;
%    data(i,2) = data(i,2)/180*pi;
%    data(i,3) = data(i,3)/180*pi;
%end

x = 0:0.01:4.99;
y = data(:,3);
plot(x,y)
%所需範圍取值
dt = 0.01;
data_h = 96;
data_t = 272;
data_length = data_t-data_h+1;
x = x(data_h:data_t);
%平滑公式
for i=data_h+1:data_t
    data(i,1) = data(i,1)*0.3 + data(i-1,1)*0.7;
    data(i,2) = data(i,2)*0.3 + data(i-1,2)*0.7;
    data(i,3) = data(i,3)*0.3 + data(i-1,3)*0.7;
end
%輸入電壓
v = 0;

U = zeros(data_length,1) + v;
thetaA = data(data_h:data_t,1);
thetaB = data(data_h:data_t,2);
phi = data(data_h:data_t,3);

dthetaA = zeros(data_length,1);
dthetaB = zeros(data_length,1);
dphi = zeros(data_length,1);
for i=1 : (data_length-1)
    dthetaA(i) = (thetaA(i+1)-thetaA(i))/dt;
    dthetaB(i) = (thetaB(i+1)-thetaB(i))/dt;
    dphi(i) = (phi(i+1)-phi(i))/dt;
end
dthetaA(data_length) = (thetaA(data_length)-thetaA(data_length-1))/dt;
dthetaB(data_length) = (thetaB(data_length)-thetaB(data_length-1))/dt;
dphi(data_length) = (phi(data_length)-phi(data_length-1))/dt;


ddthetaA = zeros(data_length,1);
ddthetaB = zeros(data_length,1);
ddphi = zeros(data_length,1);
for i=1 : (data_length-1)
    ddthetaA(i) = (dthetaA(i+1)-dthetaA(i))/dt;
    ddthetaB(i) = (dthetaB(i+1)-dthetaB(i))/dt;
    ddphi(i) = (dphi(i+1)-dphi(i))/dt;
end
ddthetaA(data_length) = (dthetaA(data_length)-dthetaA(data_length-1))/dt;
ddthetaB(data_length) = (dthetaB(data_length)-dthetaB(data_length-1))/dt;
ddphi(data_length) = (dphi(data_length)-dphi(data_length-1))/dt;

E = ddphi;
G1 = zeros(data_length,5);
for i=1 : (data_length)
    G1(i,1) = -ddthetaA(i);
    G1(i,2) = -2*cos(phi(i))*ddphi(i)+sin(2*phi(i))*sec(phi(i))*(dphi(i)^2);
    G1(i,3) = -dthetaA(i);
    G1(i,4) = dphi(i);
    G1(i,5) = U(i);
end
A = pinv(transpose(G1)*G1)*transpose(G1)*E;

G2 = zeros(data_length,5);
for i=1 : (data_length)
    G2(i,1) = -ddthetaA(i);
    G2(i,2) = -cos(phi(i))*ddphi(i);
    G2(i,3) = dthetaA(i)-dphi(i);
    G2(i,4) = sin(phi(i));
    G2(i,5) = U(i);
end
B = pinv(transpose(G2)*G2)*transpose(G2)*E;

a1 = A(1);
a2 = A(2);
a3 = A(3);
a4 = A(4);
a5 = A(5);
a6 = B(1);
a7 = B(2);
a8 = B(3);
a9 = B(4);
a10 = B(5);
%simulink結果和phi疊圖
phi = phi/180*pi;
plot(tout(1:20000),ScopeData1(1:20000,2))
hold on
plot(x,phi)

objs = instrfind;
fclose(objs);