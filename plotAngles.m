function plotAngles(t, q, plotTitle)
	q = mod(q+2*pi, 4*pi)-2*pi;
	q(q>3*pi/2) = q(q>3*pi/2)-2*pi;
	q(q<-3*pi/2) = q(q<-3*pi/2)+2*pi;
	plot(t, q);
	xlabel('Tempo (s)');
	set(gca, 'ytick', -3*pi/2:pi/2:3*pi/2);
	set(gca, 'yticklabel', {'-3pi/2', '-pi', '-pi/2', '0', 'pi/2', 'pi', '3pi/2'});
	legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
	title(plotTitle);
end

