function projeto2(p560)

close all;
robot = p560.nofriction();

n = 25;
r1 = 0.4;
r2 = 0.75;
z = -0.4;
angle = linspace(0, 2*pi, n)';
points = [r1*cos(angle), r2*sin(angle), z*ones(n, 1)];
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
imwrite(im, map, 'html\gui.gif', 'DelayTime', 0, 'LoopCount', inf);



%% Resulting robot movement
% <html>
% <img src="gui.gif" alt="Elipse no chão para o robô Puma560." align = "middle" > 
% </html>
%

end