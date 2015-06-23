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

figure;
set(gcf, 'Renderer', 'zbuffer');
robot.plot(q(1,:));
hold('on');
frame = getframe(gcf);
[im, map] = rgb2ind(frame.cdata, 256, 'nodither');
for k = 1:size(q,1)
	robot.plot(q(k, :));
	plot2(points(k, :), 'r.');
	frame = getframe(gcf);
	im(:,:,1,k) = rgb2ind(frame.cdata, map, 'nodither');
end
imwrite(im, map, 'gui.gif', 'DelayTime', 0, 'LoopCount', inf);

end