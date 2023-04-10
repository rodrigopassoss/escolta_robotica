function ZUADENTO_SIMULADOR(experimento,robo,escoltado,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos)
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% AQUI VAI O C�DIGO DO P3DX_SIM_CONTROL QUE VAI COME�AR COMO O BOT�O %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inicializa��o das vari�veis globais
global  Pdes  Mapa Mapa2 i tempo tamos;


tamos = experimento.tamos; % tempo de amostragem da simula��o em segundos
escoltado = escoltado.configuracao_inicial(experimento,tempo_max); 
for k = 1:nRobos
    robo(k) = robo(k).configuracao_inicial(experimento,escoltado,tempo_max,k); 
end
%% carregando o mapa do ambiente
A = imread(mapabmp);
A = A(:,:,1);
A = A./(max(max(A)));
A = A.*255;
% A = A(end:-1:1,:);
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

d = norm(escoltado.Pos(1:2)-Pdes);
colidiu = 0;
l_d = experimento.l_d;
fi_d = experimento.fi_d;
% Variaveis para simulacao da falha
robosComFalhas = []; status=ones(1,nRobos); 
iteracao_falha = 450;
while  (~colidiu && (i*tamos<tempo_max) && d>5)
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
    
    % Simulação de parada
    Robos = setdiff([1:nRobos],robosComFalhas);
    % Simulação dos escoltadores
    x = []; y = []; phi = [];  
    for k = 1:nRobos
                
        robo(k) = robo(k).simulacao_sensores(updateMapa2(A,robo,escoltado,nRobos,robosComFalhas,k));
%         lider = 1;
        % Definição do setPoint de distância
        n = length(Robos);
        [l_d,robo_] = min([robo(:).l_d]);  % Raio de Proteção        
        robo(k).constantes_controle(4) = cot(pi/n);
        
        Re = sqrt(min(robo(robo_).v_sensor(robo(robo_).sensorObstaculo))^2 - (l_d*sin(pi/n))^2) + l_d*cos(pi/n);

        % Setpoint de ângulo
        fi_d(Robos) = angConvert(2*pi*[0:n-1]/n);
        
        
        %%%%%%%%%%%%%%%%%%% CONTROLADOR IN�CIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %s_i = sensores no sistema de coordenadas do ambiente
        %    = sem ru�do e n�o deve ser usado pelo controlador

        %s = sensores no sistema de coordenadas do rob�
        %  = com ru�do adicionado. Esse pode ser utilizado pelo controlador.

        %s2 = sensores no sistema de coordenadas do ambiente
        %  = com ru�do adicionado. Esse pode ser utilizado pelo controlador.
        tamos_controle = 0.01; %atualizar a cada 40 ms
        
        if mod(tempo(i),tamos_controle) == 0
%                 l_d = 45;
%                 fi_d = 2*pi*(k-1)/nRobos;
%                 fi_d(k) = 2*pi*(k-1)/nRobos;
                robo(k) = robo(k).controle_e_navegacao(i,tamos_controle,tempo(i),l_d,fi_d(k),...
                                                       escoltado); 
                if ~robo(k).falhou
                    [pos,phi_] = robo(k).envia_pacote(escoltado);
                    x = [x pos(1)]; y = [y pos(2)]; phi = [phi phi_];
                end
                status(k) = ~robo(k).falhou;

        end


        %%%%%%%%%%%%%%%%%%%%% CONTROLADOR FIM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%% SIMULAÇÃO INÍCIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if habilitaDinamica==1
            robo(k) = robo(k).simulacao(tamos,i);
        else
            robo(k) = robo(k).simulacao_apenas_cinematica(tamos,i);
        end

%         dist = sqrt( (Pdes(1)-Pos(1))^2 + (Pdes(2)-Pos(2))^2 ); % atualiza a dist�ncia para o destino
         if robo(k).colidiu
             colidiu = 1;
             break;
         end
    end
    
    % Dados da comunicação broadcast
    experimento.dados_grupo = struct('x',x,...
                                     'y',y,...
                                     'phi',phi);
    % Captura de falha dos robôs
    robosComFalhas = find(status==0);
    
    % simulação de Falha 
    if i==iteracao_falha
         k = 5 + 0*randi([2 nRobos]);
         robo(k) = robo(k).simulacao_falha();  
    end
 
%     experimento.l_d = [experimento.l_d l_d];
    d = norm(escoltado.Pos(1:2)-Pdes);
    
    % PLOT DO GR�FICO "ON LINE"
    tamos_plot = 0.1; %atualizar a cada 100 ms
    if mod(tempo(i),tamos_plot) == 0
        if habilitaPlot, plot_graficos_online; end
    end 
    i*tamos
    
end



tempo = tempo(1:i);
for k = 1:nRobos
    robo(k) = robo(k).vecCorrecao(i,habilitaDinamica);
end
escoltado = escoltado.vecCorrecao(i);

%%%% Salvando os dados no arquivo
save -mat experimento.mat experimento robo escoltado tempo A Ax Ay Pdes habilitaDinamica robosComFalhas  

end