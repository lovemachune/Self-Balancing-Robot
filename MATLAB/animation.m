a = 2 ; b = 5;
p1 = [0, 0];
axis(gca,'equal');
axis([-5 5 -1 5]);
x = 0;
y = 4;
simulink_pi = [1/3,1/3,1/3,1/3,1/3,1/3,1/3,1/3,1/3,1/3];
center = [0,0];
for i=1:10
    cir = viscircles(center,1);
	lin = line([0, x],[0, y]);
    t1 = x*cos(pi*simulink_pi(i))-y*sin(pi*simulink_pi(i));
	t2 = x*sin(pi*simulink_pi(i))+y*cos(pi*simulink_pi(i));
    x =t1;
    y =t2;
    pause(1);
    delete(cir);
    delete(lin);
end
