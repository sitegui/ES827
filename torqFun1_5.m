function [ tau ] = torqFun1_5( r, t, q, qd, qRef, Kp, Kd )
	global taus tempos
	tau = (qRef-q)*Kp-qd*Kd;
	tempos = [tempos t];
	taus = [taus; tau];
end

