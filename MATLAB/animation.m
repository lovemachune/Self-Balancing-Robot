a = 2 ; b = 5;
p1 = [0, 0];
axis(gca,'equal');
axis([-5 5 -1 5]);
x = 0;
y = 4;
center = [0,0];
for i=1:500
    cir = viscircles(center,1);
	lin = line([0, 0],[x, y]);
    x = x*cos(simulink_pi(i))-y*sin(simulink_pi(i));
	y = x*sin(simulink_pi(i))-y*cos(simulink_pi(i));
    pause(0.01);
    delete(cir);
    delete(lin);
end
