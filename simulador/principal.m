clear
close all
clc

nRobos = 5; % N�mero de rob�s

%% Posi��o inicial do rob� escoltado
experimento.e_dx = 100; %100
experimento.e_dy = 80; %150 


%% Posi��o de destino desejada do rob� escoltado
experimento.dx = 900; %Posi��o de destino no eixo x [cm]
experimento.dy = 80; %Posi��o de destino no eixo y [cm]

%% Estado Inicial - controle l-fi
theta_e = atan2(experimento.dy-experimento.e_dy,experimento.dx-experimento.e_dx);
theta_i = zeros(1,nRobos);
experimento.l_ei = 50*ones(1,nRobos);
experimento.fi_ei = angConvert(2*pi*[0:nRobos-1]/nRobos);
experimento.theta_ei = theta_i;
% constantes do controlador
k1 = 0.6*ones(1,nRobos); %0.6
k2 = 0.8*ones(1,nRobos); %0.8
% Constante para a Modifica��o da Dist�ncia entre os rob�s
k3 = 0.03*ones(1,nRobos);

% Vari�veis para a Modifica��o da Dist�ncia entre os rob�s
experimento.l_d = 50; % Dist�ncia inicial
experimento.fi_d = angConvert(2*pi*[0:nRobos-1]/nRobos);
experimento.re_d = []; experimento.re_d2 = [];
experimento.rp = [];

%% Configura��es das caracter�sticas do experimento simulado (Apenas para SIMULA��O).
experimento.rbx(1:nRobos+1) = [(experimento.e_dx + experimento.l_ei.*cos(experimento.fi_ei)) experimento.e_dx]; %Posi��o inicial do rob� no eixo x [cm] (apenas para a simula��o)
experimento.rby(1:nRobos+1) = [(experimento.e_dy + experimento.l_ei.*sin(experimento.fi_ei)) experimento.e_dy]; %Posi��o inicial do rob� no eixo y [cm] (apenas para a simula��o)
experimento.ang(1:nRobos+1) = [theta_i theta_e];  %orienta��o do rob� em rela��o ao eixo x do ambiente [graus] (apenas para a simula��o)
experimento.tamos = 0.01; %tempo de amostragem em segundos (10 ms) (apenas para a simula��o)
mapabmp = 'mapa02.png'; % mapa do ambiente simulado (apenas para a simula��o)


%% Carregando o modelo Kinodin�mico do Zuadento 
% configuracao_robo
for i = 1:nRobos
    robo(i) = robo_escoltante(i,k1(i),k2(i),k3(i));

end
% Configura��es do rob� escoltado
escoltado = robo_escoltado();


%% Par�metros da simula��o
tempo_max = 60; % tempo m�ximo do experimento em segundos 
% tempo_total = tic; % controle de custo computacional (debug apenas)
%% Habilita Plot
habilitaPlot = 1;

%% Habilita Din�mica
habilitaDinamica = 0;

%% Simulador do Zuadento
ZUADENTO_SIMULADOR(experimento,robo,escoltado,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos);

% toc(tempo_total) % controle de custo computacional (debug apenas)
%% Fun��es para vizualiza��o do resultado
funcao_plotar_caminho_robo('experimento.mat',nRobos)
funcao_plotar_graficos('experimento.mat',nRobos)

