function plot2gif(numFrames, plotFrameFun, fileName)
%plot2git Create a gif file from plot frames

% Create a new figure
figure();
set(gcf, 'Renderer', 'zbuffer');

% Plot first frame, create color map and allocate image 4D array
plotFrameFun(1);
frame = getframe(gcf);
[images, map] = rgb2ind(frame.cdata, 256, 'nodither');
images(1, 1, 1, numFrames) = 0;

% Plot other frames
for n = 2:numFrames
	plotFrameFun(n);
	frame = getframe(gcf);
	images(:, :, 1, n) = rgb2ind(frame.cdata, map, 'nodither');
end
imwrite(images, map, fileName, 'DelayTime', 0, 'LoopCount', inf);

end