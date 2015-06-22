function [ ] = plotAngles( t, q )
	plot(t, wrapToPi(q));
	xlabel('Tempo (s)');
	set(gca, 'ytick', -pi:pi/2:pi);
	set(gca, 'yticklabel', {'-pi', '-pi/2', '0', 'pi/2', 'pi'});
end

