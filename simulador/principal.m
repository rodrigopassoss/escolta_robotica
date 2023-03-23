clear
close all
clc



nRobos = 5; % Número de Robôs

%% Posição inicial do robô escoltado
experimento.e_dx = 100;
experimento.e_dy = 150;


%% Posição de destino desejada do robô escoltado
experimento.dx = 900; %posição de destino no eixo x [cm]
experimento.dy = 250; %posição de destino no eixo y [cm]

%% Estado Inicial - controle l-fi
theta_e = atan2(experimento.dy-experimento.e_dy,experimento.dx-experimento.e_dx);
theta_i = zeros(1,nRobos);
experimento.l_ei = 50*ones(1,nRobos);
experimento.fi_ei = angConvert(2*pi*[0:nRobos-1]/nRobos);
experimento.theta_ei = theta_i;
% constantes do controlador
k1 = 2.6*[2.17,2.17,2.17,2.17,2.17,2.17];
k2 = 1.8*[0.12,0.12,0.12,0.12,0.12,0.12];
% Constante para a Modificação da Distância entre os robôs
k3 = 0.025*ones(1,nRobos);

% Variáveis para a Modificação da Distância entre os robôs
experimento.l_d = 80; % Distância inicial
experimento.fi_d = angConvert(2*pi*[0:nRobos-1]/nRobos);


%% Configuração das características do experimento simulado (Apenas para SIMULAÇÃO).
experimento.rbx(1:nRobos+1) = [(experimento.e_dx + experimento.l_ei.*cos(experimento.fi_ei)) experimento.e_dx]; %posição inicial do robô no eixo x [cm] (apenas para a simulação)
experimento.rby(1:nRobos+1) = [(experimento.e_dy + experimento.l_ei.*sin(experimento.fi_ei)) experimento.e_dy]; %posição inicial do robô no eixo y [cm] (apenas para a simulação)
experimento.ang(1:nRobos+1) = [theta_i theta_e];  %orientação do robô em relação ao eixo x do ambiente [graus] (apenas para a simulação)
experimento.tamos = 0.01; %tempo de amostragem em segundos (10 ms) (apenas para a simulação)
mapabmp = 'mapa02.bmp'; % mapa do ambiente simulado (apenas para a simulação)


%% Carregando o modelo Kinodinâmico do Zuadento 
% configuracao_robo
for i = 1:nRobos
    robo(i) = robo_escoltante();
    robo(i).constantes_controle = [k1(i);k2(i);k3(i)];
    robo(i).ruido = 1; %desvio padrão do ruído do sensor
    robo(i).saturacao = 200; %limite do alcance sensorial em cm
    robo(i).l_desejado = 80;
end
% configuração do robô escoltado
escoltado = robo_escoltado();
escoltado.ruido = 0; %desvio padrão do ruído do sensor
escoltado.saturacao = 200; %limite do alcance sensorial em cm

%% Parâmetros da simulação
tempo_max = 50; % tempo máximo do experimento em segundos 
% tempo_total = tic; % controle de custo computacional (debug apenas)
%% Habilita Plot
habilitaPlot = 1;

%% Habilita Dinâmica
habilitaDinamica = 0;

%% Simulador do Zuadento
ZUADENTO_SIMULADOR(experimento,robo,escoltado,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos);

% toc(tempo_total) % controle de custo computacional (debug apenas)
%% Funções para visualização do resultado
funcao_plotar_caminho_robo('experimento.mat',nRobos)
funcao_plotar_graficos('experimento.mat',nRobos)

