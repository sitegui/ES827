function [ ] = simulacao1_5( robot, tempo, q0, qd0, qRef, Kp, Kd )
	global taus tempos

	tempos = [];
	taus = [];
	[t, q] = robot.fdyn(tempo, @torqFun1_5, q0, qd0, qRef, Kp, Kd);
	plotAngles(t, q);
	title(['q^* = ', mat2str(qRef, 3)]);
	legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
	snapnow;

	plot(tempos, taus);
	title(['q^* = ', mat2str(qRef, 3)]);
	legend('\tau_1', '\tau_2', '\tau_3', '\tau_4', '\tau_5', '\tau_6');
	xlabel('Tempo (s)');
	snapnow;
end

