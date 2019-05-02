a = 2 ; b = 5;
p1 = [0, 0];
axis(gca,'equal');
axis([-15 15 -1 5]);
x = 0;
y = 4;
center_x = 0;
for i=1:100
    cir = viscircles([center_x,0],1);
	lin = line([center_x, x+center_x],[0, y]);
    x = 4*sin(simulink_phi(i,2));
	y = 4*cos(simulink_phi(i,2));
	center_x = simulink_theta(i,2);
    pause(0.1);
    delete(cir);
    delete(lin);
end
