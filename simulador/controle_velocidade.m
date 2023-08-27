
%Polos: -10,-10
% direita: kp = 0.4672, ki = 3.1179
% esquerda: kp = 0.5647, ki = 3.5935
%Polos: -20,-20
% direita: kp = 1.0908, ki = 12.4715
% esquerda: kp = 1.2834, ki = 14.3741

Kp = [0.1563;0.1540]; Ki = [0.7840;0.6601];
err = FI - obj.X;
obj.U = err.*Kp + obj.err_i.*Ki;
obj.err_i = obj.err_i + err.*tamos;

