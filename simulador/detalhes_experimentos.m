clc
clear all
close all

% Definição dos raios de segurança:
Rse = 30; % Raio de segurança do escoltado
Rsg = 25; % Raio de segurança do escoltador
Rag = 120; % Maximo alcando do robô escoltador
% Relações do número de robôs
n_max = round((2*pi)/(acos((2*Rse^2 - Rsg^2)/(2*Rse^2)))); % Número máximo de robôs na formação
n_min = ceil(pi/(asin(Rsg/Rse)));
% Para n = 4:
n = 5;
% Critério: 
    c = Rag/sin(pi/n);
% O objetivo é maximizar a cobertura:
    Re_d = c;
% A distância desejada
    Ld = Re_d*cos(pi/n) - sqrt((Re_d*cos(pi/n))^2 - Re_d^2 + Rag^2);
% Calculo de C
    C = (Rse*tan(pi/n)+Rag)/2
% Menor Raio de proteção:
    Rp_min = Rse*tan(pi/n);
    