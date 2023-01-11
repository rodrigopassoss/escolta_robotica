function ZUADENTO_SIMULADOR(experimento,robo,escoltado,mapabmp,tempo_max,habilitaPlot,nRobos)
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% AQUI VAI O C�DIGO DO P3DX_SIM_CONTROL QUE VAI COME�AR COMO O BOT�O %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inicializa��o das vari�veis globais
global  Pdes  Mapa Mapa2 i tempo tamos;


tamos = experimento.tamos; % tempo de amostragem da simula��o em segundos
for k = 1:nRobos
    robo(k) = robo(k).configuracao_inicial(experimento,tempo_max,k); 
end
escoltado = escoltado.configuracao_inicial(experimento,tempo_max); 

%% carregando o mapa do ambiente
A = imread(mapabmp);
A = A(:,:,1);
A = A./(max(max(A)));
A = A.*255;
%A = rgb2gray(A);
Mapa2 = A;
escoltado.Mapa2 = Mapa2;
% A = A(end:-1:1,:); % utilizada no plot para parecer a imagem no SC padr�o
[Ay , Ax] = find(A~=255);
Mapa = [Ax,Ay]';
escoltado.Mapa = Mapa;

%% inicialização da posição de destino do robô
Pdes = [experimento.dx ; experimento.dy ];
escoltado.Pdes = Pdes;

% inicializa��o das vari�veis do loop while
tempo = 0:tamos:tempo_max;  % controle de tempo
i = 0;  % contador

colidiu = 0;
while  (~colidiu && (i*tamos<tempo_max))
      % distancia maior que 5 cm ou vlin maior q 5 cm/s ou vrot maior que 0.1 rad/s
    tic     
    % atualiza��o das vari�veis de controle de tempo
    i = i+1;          
    tempo(i+1) = i*tamos;   
    
    % Simulação do Robô escoltado
    escoltado = escoltado.simulacao_sensores(updateMapa(A,robo,nRobos));
    escoltado = escoltado.controle_e_navegacao(i);
    escoltado = escoltado.simulacao(tamos,i);
    if escoltado.colidiu, colidiu = 1; end
    
    % Simulação dos escoltadores
    for k = 1:nRobos
                
        robo(k) = robo(k).simulacao_sensores(updateMapa2(A,robo,escoltado,nRobos,k));
        
        %%%%%%%%%%%%%%%%%%% CONTROLADOR IN�CIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %s_i = sensores no sistema de coordenadas do ambiente
        %    = sem ru�do e n�o deve ser usado pelo controlador

        %s = sensores no sistema de coordenadas do rob�
        %  = com ru�do adicionado. Esse pode ser utilizado pelo controlador.

        %s2 = sensores no sistema de coordenadas do ambiente
        %  = com ru�do adicionado. Esse pode ser utilizado pelo controlador.
        tamos_controle = 0.04; %atualizar a cada 40 ms
        if mod(tempo(i),tamos_controle) == 0
                fi_d = (2*pi*(1-k)/nRobos); 
                l_d = 45;
                robo(k) = robo(k).controle_e_navegacao(i,tamos_controle,tempo(i),fi_d,l_d,escoltado); 
        end

        %%%%%%%%%%%%%%%%%%%%% CONTROLADOR FIM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%% SIMULAÇÃO INÍCIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        robo(k) = robo(k).simulacao(tamos,i);

%         dist = sqrt( (Pdes(1)-Pos(1))^2 + (Pdes(2)-Pos(2))^2 ); % atualiza a dist�ncia para o destino
         if robo(k).colidiu
             colidiu = 1;
             break;
         end
    end
    
    % Atualiza mapa
    
    % PLOT DO GR�FICO "ON LINE"
    tamos_plot = 0.1; %atualizar a cada 100 ms
    if mod(tempo(i),tamos_plot) == 0
        if habilitaPlot, plot_graficos_online; end
    end 
    i*tamos
    
end

tempo = tempo(1:i);
for k = 1:nRobos
    robo(k) = robo(k).vecCorrecao(i);
end
escoltado = escoltado.vecCorrecao(i);

%%%% Salvando os dados no arquivo
save -mat experimento.mat robo escoltado tempo A Ax Ay Pdes    

end