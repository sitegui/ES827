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
	plotAngles(t, q);
	title(['q0 = ', mat2str(q0, 3)]);
	legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
	snapnow;
end

%% 1.4.a - Simulação livre com atrito viscoso
tempo = 30;
for q0 = [qz; 0, 1e-6, 0, 0, 0, 0]'
	[t, q, qd] = robot.fdyn(tempo, qz, q0, qd0);
	K = zeros(size(q, 1));
	for i = 1:size(q, 1)
		M = robot.inertia(q(i, :));
		K(i) = 1/2*qd(i, :)*M*qd(i, :)';
	end
	plotAngles(t, q);
	title(['q0 = ', mat2str(q0, 3)]);
	legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
	snapnow;
	
	plot(t, K);
	snapnow;
end

%% 1.4.b - Simulação livre com atrito viscoso
tempo = 30;
robot2 = robot.nofriction('all');

for q0 = [qz; 0, 1e-6, 0, 0, 0, 0]'
	[t, q, qd] = robot2.fdyn(tempo, qz, q0, qd0);
	K = zeros(size(q, 1));
	for i = 1:size(q, 1)
		M = robot2.inertia(q(i, :));
		K(i) = 1/2*qd(i, :)*M*qd(i, :)';
	end
	plotAngles(t, q);
	title(['q0 = ', mat2str(q0, 3)]);
	legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
	snapnow;
	
	plot(t, K);
	snapnow;
end

%% 1.5 - Simulação de controlador PD
Kp = diag([50, 50, 50, 50, 50, 60]);
Kd = diag([20, 20, 20, 20, 20, 22]);
tempo = 5;
robot1_5 = robot;
robot1_5.gravity = [0; 0; 0];
run1_5 = @(q0, qRef)simulacao1_5(robot1_5, tempo, q0, qz, qRef, Kp, Kd);
run1_5([0,    0,     0, 0, 0, 0], [pi/2,  0, -pi/2,  pi,  pi/2, -pi]);
run1_5([0,   pi, -pi/2, 0, 0, 0],   [pi,  0,     0,  pi, -pi/2, 0]);
run1_5([0, pi/2, -pi/2, 0, 0, 0],  [-pi, pi,   -pi, -pi, -pi/2, pi]);