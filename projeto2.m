function projeto2(p560)

close all;
robot = p560.nofriction();

n = 25;
r = 0.5;
z = -0.4;
angle = linspace(0, 2*pi, n)';
points = [r*cos(angle), r*sin(angle), z*ones(n, 1)];
T = transl(points);
q = robot.ikine(T);

	function plotFrame(n)
		robot.plot(q(n, :));
		hold('on');
		plot2(points(n, :), 'r.');
	end

plot2gif(size(q, 1), @plotFrame, '2.1.gif');

end