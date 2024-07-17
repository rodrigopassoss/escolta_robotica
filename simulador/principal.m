clear
close all
clc

nRobos = 5; % Número de robôs

%% Posição inicial do robô escoltado
experimento.e_dx = 100; %100
experimento.e_dy = 80; %150 


%% Posição de destino desejada do robô escoltado
experimento.dx = 900; %Posição de destino no eixo x [cm]
experimento.dy = 80; %Posição de destino no eixo y [cm]

%% Estado Inicial - controle l-fi
theta_e = atan2(experimento.dy-experimento.e_dy,experimento.dx-experimento.e_dx);
theta_i = zeros(1,nRobos);
experimento.l_ei = 50*ones(1,nRobos);
experimento.fi_ei = angConvert(2*pi*[0:nRobos-1]/nRobos);
experimento.theta_ei = theta_i;
% constantes do controlador
k1 = 0.6*ones(1,nRobos); %0.6
k2 = 0.8*ones(1,nRobos); %0.8
% Constante para a Modificação da Distância entre os robôs
k3 = 0.03*ones(1,nRobos);

% Variáveis para a Modificação da Distância entre os robôs
experimento.l_d = 50; % Distância inicial
experimento.fi_d = angConvert(2*pi*[0:nRobos-1]/nRobos);
experimento.re_d = []; experimento.re_d2 = [];
experimento.rp = [];

%% Configurações das características do experimento simulado (Apenas para SIMULAÇÃO).
experimento.rbx(1:nRobos+1) = [(experimento.e_dx + experimento.l_ei.*cos(experimento.fi_ei)) experimento.e_dx]; %Posição inicial do robô no eixo x [cm] (apenas para a simulação)
experimento.rby(1:nRobos+1) = [(experimento.e_dy + experimento.l_ei.*sin(experimento.fi_ei)) experimento.e_dy]; %Posição inicial do robô no eixo y [cm] (apenas para a simulação)
experimento.ang(1:nRobos+1) = [theta_i theta_e];  %orientação do robô em relação ao eixo x do ambiente [graus] (apenas para a simulação)
experimento.tamos = 0.01; %tempo de amostragem em segundos (10 ms) (apenas para a simulação)
mapabmp = 'mapa02.png'; % mapa do ambiente simulado (apenas para a simulação)


%% Carregando o modelo Kinodinâmico do Zuadento 
% configuracao_robo
for i = 1:nRobos
    robo(i) = robo_escoltante(i,k1(i),k2(i),k3(i));

end
% Configurações do robô escoltado
escoltado = robo_escoltado();


%% Parâmetros da simulação
tempo_max = 60; % tempo máximo do experimento em segundos 
% tempo_total = tic; % controle de custo computacional (debug apenas)
%% Habilita Plot
habilitaPlot = 1;

%% Habilita Dinâmica
habilitaDinamica = 0;

%% Simulador do Zuadento
ZUADENTO_SIMULADOR(experimento,robo,escoltado,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos);

% toc(tempo_total) % controle de custo computacional (debug apenas)
%% Funções para vizualização do resultado
funcao_plotar_caminho_robo('experimento.mat',nRobos)
funcao_plotar_graficos('experimento.mat',nRobos)

