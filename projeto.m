clear all;
mdl_puma560;
robot = p560.nofriction();

%% 1.3 - Simulação em malha aberta
% Com torque pra compensar a gravidade

tau = robot.rne(qs, qz, qz);
tempo = 5;
torqFun = @(r, t, q, qd)tau;
qd0 = qz;

qs2 = qs;
qs2(3) = qs2(3) + 0.05;
for q0 = [qz; qr; qs; qs2]'
	[t, q, qd] = robot.fdyn(tempo, torqFun, q0, qd0);
	plot(t, q);
	title(['q0 = ', mat2str(q0)]);
	legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
	xlabel('Tempo (s)');
	snapnow;
end