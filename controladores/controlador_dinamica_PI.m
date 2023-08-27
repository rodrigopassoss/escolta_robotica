clc
close all
clear all
% Controlador Dinâmica
s = tf('s');
a = 32.0731; b = 5.0145;
% a = 27.8278; b = 4.2860;
H = a/(s+b);

for i=b

po = i;
kp = (2*po-b)/a;
ki = (po^2)/a;

%Polos: -10,-10
% direita: kp = 0.4672, ki = 3.1179
% esquerda: kp = 0.5647, ki = 3.5935
%Polos: -20,-20
% direita: kp = 1.0908, ki = 12.4715
% esquerda: kp = 1.2834, ki = 14.3741

C = kp + ki/s; % PI

G = minreal(C*H/(1+C*H));

p = pole(G)
% Simulação 1: Resposta ao degrau
figure
subplot(211)
step(G);
subplot(212)
step((a/b)*minreal(C/(1+C*H)));

% Simulação 2: Resposta ao degrau
err_i = 0; % erro integral
u_ = 0; 
dt = 0.001;
y = 0; % saida (o próprio estado do sistema)
y_ = 0;
duracao = 2; N = round(duracao/dt);
r = (a/b)*(1:(N+1)).^0;
k = 1;
for i = 1:N
   err = r(i) - y;
   % sinal controle 1
   u = kp*err + ki*err_i;
   err_i = err_i + err*dt;

   % Simu Controle 1
   dy = -b*y + a*u;
   y = y + dy*dt;
   y_ = [y_ y]; u_ = [u_ u];
end

end

t = 0:dt:duracao;
figure
subplot(211)
grid on
hold on
plot(t,y_)
plot(t,r,'--k')
subplot(212)
grid on
hold on
plot(t,u_)



