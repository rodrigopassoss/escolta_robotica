function ZUADENTO_SIMULADOR(experimento,robo,escoltado,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos)
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inicialização das variáveis globais
global  Pdes  Mapa Mapa2 i tempo tamos;


tamos = experimento.tamos; % tempo de amostragem da simualção em segundos
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
% A = A(end:-1:1,:); % utilizada no plot para parecer a imagem no SC padrão
[Ay , Ax] = find(A~=255);
Mapa = [Ax,Ay]';
escoltado.Mapa = Mapa;

%% inicialização da posição de destino do robô
Pdes = [experimento.dx ; experimento.dy ];
escoltado.Pdes = Pdes;

% inicialização das variáveis do loop while
tempo = 0:tamos:tempo_max;  % controle de tempo
i = 0;  % contador

d = norm(escoltado.Pos(1:2)-Pdes);
colidiu = 0;
l_d = experimento.l_d;
fi_d = experimento.fi_d;
% Variaveis para simulacao da falha
robosComFalhas = []; status=ones(1,nRobos); 
iteracao_falha = 1e10; %450 
robo_falhado = 6;
while  (~colidiu && (i*tamos<tempo_max) && d>5)
      % distancia maior que 5 cm ou vlin maior q 5 cm/s ou vrot maior que 0.1 rad/s
    tic     
    % atualização das variáveis de controle de tempo
    i = i+1;          
    tempo(i+1) = i*tamos;   
    
    % simulação do robô escoltado
    escoltado = escoltado.simulacao_sensores(updateMapa(A,robo,nRobos));
    escoltado = escoltado.controle_e_navegacao(i);
    escoltado = escoltado.simulacao(tamos,i);
    if escoltado.colidiu, colidiu = 1; end
    
    % simulação de parada
    Robos = setdiff([1:nRobos],robosComFalhas);
    % simulação dos escoltadores
    x = []; y = []; phi = [];  
    for k = 1:nRobos
                
        robo(k) = robo(k).simulacao_sensores(updateMapa2(A,robo,escoltado,nRobos,robosComFalhas,k));
%         lider = 1;
        n = length(Robos);
        robo(k).constantes_controle(4) = n; % Número de Robôs
        %--- Definição dos setPoints
        % Setpoit de distância
        [l_d,~] = min([robo(Robos).l_d]);  
        % Setpoit de Angulo
        deltas = [robo(Robos).delta_fi_d];
                % -- Ponderação
                l_obst = [robo(Robos).l_obst];
                pesos = 1./(l_obst);
                pesos = pesos/sum(pesos);
                % --
        delta_fi_d = pesos*deltas';
        fi_d(k) = angConvert((2*pi*(k-1)/n) + delta_fi_d);
%          fi_d(k) = angConvert((2*pi*(k-1)/n) + robo(k).delta_fi_d);
        % ------

        
        %%%%%%%%%%%%%%%%%%% CONTROLADOR INÍCIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %s_i = sensores no sistema de coordenadas do ambiente
        %    = sem ruído e não deve ser usado pelo controlador

        %s = sensores no sistema de coordenadas do robô
        %  = com ruído adicionado. Esse pode ser utilizado pelo controlador.

        %s2 = sensores no sistema de coordenadas do ambiente
        %  = com ruído adicionado. Esse pode ser utilizado pelo controlador.
        tamos_controle = 0.01; %atualizar a cada 40 ms
        
        if mod(tempo(i),tamos_controle) == 0
%                 l_d = 45;
%                 fi_d = 2*pi*(k-1)/nRobos;
%                 fi_d(k) = 2*pi*(k-1)/nRobos;
                robo(k) = robo(k).controle_e_navegacao(i,tamos_controle,tempo(i),l_d,fi_d(k),...
                                                       escoltado,habilitaDinamica); 
%                 if ~robo(k).falhou
%                     [pos,phi_] = robo(k).envia_pacote(escoltado);
%                     x = [x pos(1)]; y = [y pos(2)]; phi = [phi phi_];
%                 end
                status(k) = ~robo(k).falhou;

        end


        %%%%%%%%%%%%%%%%%%%%% CONTROLADOR FIM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%% SIMULAÃ‡ÃƒO INÃ?CIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if habilitaDinamica==1
            robo(k) = robo(k).simulacao(tamos,i);
        else
            robo(k) = robo(k).simulacao_apenas_cinematica(tamos,i);
        end

         if robo(k).colidiu
            robo(k) = robo(k).simulacao_falha();
         end
         
         % simulação da comunicação
         dists_to_robots = []; outros_robos = setdiff(Robos,k);
         for l = outros_robos
             dists_to_robots = [dists_to_robots ...
                                sqrt((robo(k).Pos(1) - robo(l).Pos(1))^2 ...
                                + (robo(k).Pos(2) - robo(l).Pos(2))^2)];
         end 
         idc_vizinhos = find(dists_to_robots<robo(k).Rc);
         robo(k) = robo(k).comunicacao(robo,outros_robos(idc_vizinhos));
         % Parda de comunicação
         if min(dists_to_robots)>robo(k).Rc
            robo(k) = robo(k).simulacao_falha();
         end
         
         if escoltado.colidiu
             colidiu = 1;
             break;
         end
    end
    
    % Dados da comunicação broadcast
%     experimento.dados_grupo = struct('x',x,...
%                                      'y',y,...
%                                      'phi',phi);
    % Captura de falha dos robôs
    robosComFalhas = find(status==0);
    
    % simulação de Falha 
    if i==iteracao_falha
%          k = Robos(randi([2 length(Robos)]));
         k = robo_falhado;
         robo(k) = robo(k).simulacao_falha();
         iteracao_falha = 3500;
         robo_falhado = 5;
         if habilitaPlot, plot_graficos_online; end
    end
 
%     experimento.l_d = [experimento.l_d l_d];
    d = norm(escoltado.Pos(1:2)-Pdes);
    
    Re = sqrt((robo(1).r_p).^2 - (l_d.*sin(pi/n)).^2) + l_d.*cos(pi/n);
%     Re2 = sqrt((robo(1).r_p).^2 - (l_d.*sin((2*pi-alfa)/(2*(n-1)))).^2) + l_d.*cos((2*pi-alfa)/(2*(n-1)));
    if imag(Re)~=0, Re=NaN; end
    experimento.re_d = [experimento.re_d Re];
%     experimento.re_d2 = [experimento.re_d2 Re2];
    experimento.rp = [experimento.rp robo(1).r_p];
    
    % PLOT DO GRÁFICO "ON LINE"
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