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


x = 0:0.01:4.99;
y = data(:,3);
plot(x,y)
%所需範圍取值
x = 0:0.01:4.99;
dt = 0.01;
data_h = 40;
data_t = 48;  %data取到t+2 ddata取到t+1
data_length = data_t-data_h+1;
x = x(data_h:data_t);
%輸入電壓
v = 5;
%{
%平滑公式
for i=data_h+1:data_t
    data(i,1) = data(i,1)*0.3 + data(i-1,1)*0.7;
    data(i,2) = data(i,2)*0.3 + data(i-1,2)*0.7;
    data(i,3) = data(i,3)*0.3 + data(i-1,3)*0.7;
end
%}
U = zeros(data_length,1) + v;
thetaB = (data(data_h:data_t+2,1)/180)*pi;
thetaA = (data(data_h:data_t+2,2)/180)*pi;
phi = (data(data_h:data_t+2,3)/180)*pi;

dthetaA = zeros(data_length+1,1);
dthetaB = zeros(data_length+1,1);
dphi = zeros(data_length+1,1);
for i=1 : (data_length+1)
    dthetaA(i) = (thetaA(i+1)-thetaA(i))/dt;
    dthetaB(i) = (thetaB(i+1)-thetaB(i))/dt;
    dphi(i) = (phi(i+1)-phi(i))/dt;
end

ddthetaA = zeros(data_length,1);
ddthetaB = zeros(data_length,1);
ddphi = zeros(data_length,1);
for i=1 : (data_length)
    ddthetaA(i) = (dthetaA(i+1)-dthetaA(i))/dt;
    ddthetaB(i) = (dthetaB(i+1)-dthetaB(i))/dt;
    ddphi(i) = (dphi(i+1)-dphi(i))/dt;
end

thetaB = thetaB(1:data_length);
thetaA = thetaA(1:data_length);
phi = phi(1:data_length);
dthetaA = dthetaA(1:data_length);
dthetaB = thetaB(1:data_length);
dphi = phi(1:data_length);

%%%Original Matrix%%%
%{
for i=1 : (data_length)
    G1(i,1) = -ddthetaA(i);
    G1(i,2) = -2*cos(phi(i))*ddphi(i)+sin(2*phi(i))*sec(phi(i))*(dphi(i)^2);
    G1(i,3) = -dthetaA(i);
    G1(i,4) = dphi(i);
    G1(i,5) = U(i);
end
%}

%%%%%%%%%%%For 0V%%%%%%%%%%%%%%%%%
%{
%%%G1(1) G1(3) G1(5) (a1 a3 a5) can be ignore because theta=0 and V=0%%
E = ddphi;
G1 = zeros(data_length,2);
for i=1 : (data_length)
    G1(i,1) = -2*cos(phi(i))*ddphi(i)+sin(2*phi(i))*sec(phi(i))*(dphi(i)^2);
    G1(i,2) = dphi(i);
end
A = inv(transpose(G1)*G1)*transpose(G1)*E;

%%%G2(1) G2(2) G2(5) (a6 a7 a10) can be ignore  and G2(3) = -dphi (a8)%%%
G2 = zeros(data_length,2);
for i=1 : (data_length)
    G2(i,1) = -dphi(i);
    G2(i,2) = sin(phi(i));
end
B = inv(transpose(G2)*G2)*transpose(G2)*E;
%}
%%%%%%%%%%%Others%%%%%%%%%%%%
%{
%%%已知 a2(A(1)) a4(A(2)) 求 a1 a3 a5
E = zeros(dataG1
_length,1);
G1 = zeros(data_length,3);
for i=1 : (data_length)
    E = ddphi(i) + A(1)*2*cos(phi(i))*ddphi(i)+sin(2*phi(i))*sec(phi(i))*(dphi(i)^2) - A(2)*dphi(i);
    G1(i,1) = -ddthetaA(i);
    G1(i,2) = -dthetaA(i);
    G1(i,3) = U(i);
end
A = inv(transpose(G1)*G1)*transpose(G1)*E;

%%%已知 a8(B(1)) a9(B(2)) 求 a6 a7 a10
G2 = zeros(data_length,3);
for i=1 : (data_length)
    E = ddphi(i) + B(1)*dphi(i) -B(2)*sin(phi(i));
    G2(i,1) = -ddthetaA(i);
    G2(i,2) = -cos(phi(i))*ddthetaA(i);
    G2(i,3) = -U(i);
end
B = inv(transpose(G2)*G2)*transpose(G2)*E;
%}

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
x = 0:0.01:1.76;
plot(out.tout(1:2000),out.ScopeData1(1:2000,2))
hold on
plot(x,phi)

%state equation
a = 1+2*a2;
b = a6+a7;
k1 = a-a1/b;
k2 = (1/a)-b;
m1 = -a1*a9/b;
m2 = a4+a1*a8/b;
m3= -a3-a1*a8/b;
m4 = a5+a1*a10/b;

n1 = -a9;
n2 = a8+a4/a;
n3 = -a8-a3/a;
n4 = a10+a5/a;

matrixA = [0 1 0 0;
           m1/k1 m2/k1 0 m3/k1;
           0 0 0 1;
           n1/k2 n2/k2 0 n3/k2];
matrixB = [0;m4/k1;0;n4/k2];
matrixC = [1 0 1 0];
matrixD = 0;
[G,H] = ss2tf(matrixA, matrixB, matrixC, matrixD,1);
rlocus(G,H)

objs = instrfind;
fclose(objs);