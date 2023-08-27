clc
clear all
close all

% Defini��o dos raios de seguran�a:
Rse = 30; % Raio de seguran�a do escoltado
Rsg = 25; % Raio de seguran�a do escoltador
Rag = 120; % Maximo alcando do rob� escoltador
% Rela��es do n�mero de rob�s
n_max = round((2*pi)/(acos((2*Rse^2 - Rsg^2)/(2*Rse^2)))); % N�mero m�ximo de rob�s na forma��o
n_min = ceil(pi/(asin(Rsg/Rse)));
% Para n = 4:
n = 5;
% Crit�rio: 
    c = Rag/sin(pi/n);
% O objetivo � maximizar a cobertura:
    Re_d = c;
% A dist�ncia desejada
    Ld = Re_d*cos(pi/n) - sqrt((Re_d*cos(pi/n))^2 - Re_d^2 + Rag^2);
% Calculo de C
    C = (Rse*tan(pi/n)+Rag)/2
% Menor Raio de prote��o:
    Rp_min = Rse*tan(pi/n);
    