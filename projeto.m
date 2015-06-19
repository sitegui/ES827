clear all;
mdl_puma560;
robot = p560.nofriction();

% Condições iniciais
tempo = 10;
q0 = qs;
qd0 = qz;
q0(3) = q0(3)+5*pi/180;

%t = (0:.056:5)';
%q_dmd = jtraj(qz, qr, t);
%qt = [t, q_dmd];
tau = robot.rne(qs, qz, qz);
torqFun = @(r, t, q, qd)tau;
[t, q, qd] = robot.fdyn(tempo, torqFun, q0, qd0);
plot(t, q);