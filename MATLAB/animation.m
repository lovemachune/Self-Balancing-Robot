a = 2 ; b = 5;
p1 = [0, 0];
axis(gca,'equal');
axis([-5 5 -1 5]);
x = 0;
y = 4;
center_x = 0;
simulink_pi = [90*pi/180,80*pi/180,70*pi/180,60*pi/180,50*pi/180,40*pi/180,30*pi/180,20*pi/180,10*pi/180,0];
simulink_theta = [60*pi/180,60*pi/180,60*pi/180,60*pi/180,60*pi/180,60*pi/180,60*pi/180,60*pi/180,60*pi/180,60*pi/180];
for i=1:10000
    cir = viscircles([center_x,0],1);
	lin = line([center_x, x],[0, y]);
    x = 4*sin(out.simulink_phi(i));
	y = 4*cos(out.simulink_phi(i));
	center_x = center_x+sin(out.simulink_theta(i));
    pause(0.00001);
    delete(cir);
    delete(lin);
end
