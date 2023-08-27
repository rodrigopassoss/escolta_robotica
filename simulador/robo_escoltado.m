classdef robo_escoltado
    % robo classe que representa o rob�
    
    properties
    % Caracter�sticas f�sicas do rob�    
        r  % 5.3000
        L  % 5.6250
        Modelo  % [2�2 ss]
        raio  % 8
        Modcin  % [2�2 double]
        ganhoft  % [2�1 double]
        Vmax  % 34.1570
        Wmax  % 6.0724
        ruido  %  1
        saturacao  % 200
    % Raio de Seguran�a
        Rs
    % Informa��es sobre o mapa
        Mapa
        Mapa2
    % Informa��es da tarefa do rob�
        Pos 
        hPos
        Pdes
        v_sensor
        s_i
        s
        s2 
        angs
        colidiu
        path   % Caminho Planejado
        e_path % Caminho Executado
    % Sinais de Controle
        U    % Contem o sinal de controle da roda esquerda e direita
        Ud   % Sinal de controle desejado
        X    % estado din�mico do sistema
    % Informa��es para o plot
        plotInfo
        plt
    end
    
    methods
        function obj = robo_escoltado()
            %%% Par�metros de configura��o das caracter�sticas do rob�.
            % Wd/FId = A/(s+B); %velocidade angula na sa�da em rad/s
            %    -15.66
            %   --------
            %   s + 4.87
            %    -15.11
            %   ---------
            %   s + 5.014
            % We/FIe = C/(s+D); %velocidade angula na sa�da em rad/s
            %    17.75
            %   ---------
            %   s + 5.833
            %    13.11
            %   ---------
            %   s + 4.286
            obj.r = 5.3; %raio da roda
            obj.L = 11.25/2; %dist�ncia da roda ao centro do rob� (metade da distancia entre as rodas)
            k_w_to_fi = 2*obj.L/obj.r;
            A = 15.11;
            A_ = A*k_w_to_fi;
            B = 5.014;
            C = -13.11;
            C_ = C*(-k_w_to_fi);
            D = 4.286;
            Ma = [-B 0 ; 0 -D];
            Mb = [A_ 0 ; 0 C_];
            Mc = [obj.r/2 , obj.r/2 ; obj.r/(2*obj.L) -obj.r/(2*obj.L)];
            Md = [0 0 ; 0 0];

            obj.Modelo = ss(Ma,Mb,Mc,Md);

            % U = [U_d ; U_e] entradas de -1 a 1
            % X = [FI_d ; FI_e] estado
            % Y = [V ; W] sa�das
            obj.Rs = 30;

            obj.raio = 8; %raio do rob� para verificar colis�o e plotar o rob� [cm]
            obj.Modcin = Mc; %modelo cinem�tico direto
            obj.ganhoft = [B/A_ ; D/C_];
            % U = inv(obj.Modcin)*[V ; W];

            fi = [1 ; 1]./[obj.ganhoft];
            aux = obj.Modcin*(fi);
            obj.Vmax = aux(1); %c�lculo da vellcidade linear m�xima a partir do modelo identificado
            fi = [1 ; -1]./[obj.ganhoft];
            aux = obj.Modcin*(fi);
            obj.Wmax = aux(2); %c�lculo da vellcidade angular m�xima a partir do modelo identificado
        
        end
        
        function obj = controle_e_navegacao(obj,iteracao)
        
            %%% Controlador Deliberativo com a RRT
            if iteracao == 1
                passo = 50; rRobo = 45; 
                sMap = size(obj.Mapa2');
                [~, path2] = RRT(obj.Mapa,obj.Pdes', obj.Pos', passo, sMap, rRobo) ;

            %%%%%% Suaviza��o do caminho
               %%% Etapa 01: Elimina��o de redund�ncias do caminho
               Nn = length(path2(:,1));
               for k = 1:Nn-1
                    if k > Nn, break; end
                    for l = 0:Nn-1
                        if (~collision_detection(obj.Mapa',path2(end-l,:),path2(k,:),rRobo) && ((k+1) < (Nn-l-1)))
                            path2((k+1):(end-l-1),:)=[];
                            break;
                        end
                    end
                    Nn = length(path2(:,1));   % Atualiza o tamanho do caminho
               end
               %%% Etapa 02: Interpola��o do Caminho:
               obj.path = [];
               for k = 1:Nn-1
                   d = passo/norm(path2(k+1,:)-path2(k,:)); B = [0:d:1;0:d:1]'; 
                   obj.path = [obj.path; B.*path2(k+1,:) + (B.^0-B).*path2(k,:)];
               end
               %%% Etapa 03: Filtragem com Filtro Passa baixas:
               % H = A/(s + A), A = 0.5;
               % H = Y/U => (Y(k+1) - Y)/dt = -AY + AU
               %         =>  Y(k+1) = (1-dtA)Y + dtAU
               A = 0.9; 
               ordem = 5;
               for n = 1:ordem
               u = obj.path;
               for k = 2:length(u)
                   obj.path(k,:) = A*u(k,:) + (1-A)*obj.path(k-1,:);
               end
               end
               obj.path = [obj.path;path2(end,:)]; % Conecta o caminho filtrado ao objetivo;
               obj.e_path = [];
            end
            %%%%%% Seguidor de trajet�ria
            [~, I] = min(((obj.path(:,1)-obj.Pos(1)).^2 + (obj.path(:,2)-obj.Pos(2)).^2).^0.5);
            passo = 1;
            if I >= (length(obj.path)-passo), I = length(obj.path)-passo; end
            p_des = obj.path(I+passo,1:2);

            %%% Norma do vetor gradiente (d)
            d = norm(p_des-obj.Pos(1:2)); %dist?ncia at? o destino
            obj.e_path = [obj.e_path obj.Pos(1:2)];
            %%% angula��o do vetor gradiente (theta_e)
            theta_d = atan2(p_des(2)-obj.Pos(2),p_des(1)-obj.Pos(1)); % ?ngulo de destino de -pi a pi
            theta_e = theta_d - obj.Pos(3);

            % converte theta_e para -pi a pi
            if theta_e > pi, theta_e = theta_e - 2*pi; end
            if theta_e < -pi, theta_e = theta_e + 2*pi; end
            wmax = 5;kw = 0.55; %5
            W = wmax*tanh(kw*theta_e);
            % converte theta_e para -pi/2 a pi/2
            if theta_e > pi/2, theta_e = pi/2; end
            if theta_e < -pi/2, theta_e = -pi/2; end
            vmax = 15;kv = 2.5; %20
            V = vmax*cos(theta_e);

            if norm(obj.Pos(1:2)-obj.Pdes)<3, V = 0; W = 0; end

            
            %%% sa�da da fun��o de controle deve ser U = [U_d ; U_e] entrada
            FI = obj.Modcin\[V ; W]; %FI = [FI_d ; FI_e];
            obj.U = [obj.ganhoft].*FI;
            
            obj.Ud = obj.U;
            % Satura��o do sinal de controle
            if abs(obj.U(1)) > 1
                obj.U(1) = sign(obj.U(1));
            end
            if abs(obj.U(2)) > 1
                obj.U(2) = sign(obj.U(2));
            end
            
        end
        
        function [Ksir_real, X] = dinamica_zuadento(obj,Tamos)
               
                U = [obj.U obj.U obj.U]; %[Ud ; Ue]
            % % %     U = U'; % descomentar essa linha pra usar no octave
                %Xp = Ma*X + Mb*U
                % Y = Mc*X + Md*U
                [Y,T,X] = lsim(obj.Modelo,U ,[0:Tamos:2*Tamos],obj.X);
                X = X(2,:)';
                Ksir_real = [Y(2,1) ; 0 ; Y(2,2)];
        end
        
        function obj = simulacao(obj,tamos,iteracao)
                R = [cos(obj.Pos(3)) sin(obj.Pos(3)) 0 ; -sin(obj.Pos(3)) cos(obj.Pos(3)) 0 ; 0 0 1]; % matriz de rota??o
                [Ksir_real, X] = dinamica_zuadento(obj,tamos); %retorna as velocidades V e W reais no SC do rob?
                obj.X = X;
                Ksi_I = R\Ksir_real; % coloca as velocidades V e W reais no SC do ambiente
                obj.Pos = obj.Pos + Ksi_I*tamos; % atualiza??o da posi??o do rob? (integra??o no SC do ambiente)
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = Ksir_real(1);
                Wmedido = Ksir_real(3);

                FIreal = obj.Modcin\[Vmedido ; Wmedido];
                Ureal = [obj.ganhoft].*FIreal;

                fi = obj.Ud./[obj.ganhoft]; % fi desejado 
                Ksir_d = obj.Modcin*(fi); % converte para comandos de velocidade (apenas para registro e plots futuros)

                
                obj.plotInfo.Pvel(:,iteracao+1) = Ksir_d; % atualiza o vetor das velocidades (comandos) do rob? durante o experimento.
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
                obj.plotInfo.Pu(:,iteracao+1) = obj.Ud; % U = [U_d ; U_e] de -1 a 1 % valor desejado
                obj.plotInfo.Pu_real(:,iteracao+1) = Ureal; % U = [U_d ; U_e] de -1 a 1 % valor real
                obj.plotInfo.Pfi(:,iteracao+1) = fi; % U = [Fi_d ; Fi_e] % valor desejado de fi
                obj.plotInfo.Pfi_real(:,iteracao+1) = FIreal; % U = [Fi_d ; Fi_e] % valor real de fi
                
        end
        
        function obj = simulacao_sensores(obj,A)
           
            %%% tratamento dos sensores (antiga fun��o inicio)
            Ps_i = obj.s_i;
            [ymaxA , xmaxA] = size(A);
            smax = obj.saturacao*(ones(1,length(obj.angs)));
            smax = smax + obj.raio;
            %s � um vetor cujos elementos representam o ponto da leitura m�xima (a priori) para
            %cada um dos sensores no sistema de coordenadas do rob�
            obj.s = [smax.*cos(obj.angs) ; smax.*sin(obj.angs)]; % vetor no SC do rob� (ainda saturado)    
            Ri = [cos(obj.Pos(3)) -sin(obj.Pos(3)); sin(obj.Pos(3)) cos(obj.Pos(3))]; % matriz de rota��o
            %s_i � o vetor s colocado no sistema de coordenadas do ambiente (sem ru�do) (ainda saturado)
            obj.s_i = Ri*obj.s;
            obj.s_i(1,:) = obj.s_i(1,:) + obj.Pos(1);
            obj.s_i(2,:) = obj.s_i(2,:) + obj.Pos(2);

            pos_sensor = [obj.raio.*cos(obj.angs) ; obj.raio.*sin(obj.angs)];
            pos_sensor = Ri*pos_sensor;
            pos_sensor(1,:) = pos_sensor(1,:) + obj.Pos(1);
            pos_sensor(2,:) = pos_sensor(2,:) + obj.Pos(2);

            for k=1:length(obj.angs)
                Vsx = round(linspace(pos_sensor(1,k),obj.s_i(1,k),obj.saturacao));
                Vsy = round(linspace(pos_sensor(2,k),obj.s_i(2,k),obj.saturacao));
                Vs = [Vsx ; Vsy];
                for j=1:obj.saturacao
                    if (Vs(1,j)>0 && Vs(1,j)<=xmaxA && Vs(2,j)>0 && Vs(2,j)<=ymaxA) % se a leitura esta dentro do mapa
                        if A(Vs(2,j),Vs(1,j)) ~= 255
                            obj.s_i(1,k) = Vs(1,j);
                            obj.s_i(2,k) = Vs(2,j);
                            break;
                        end
                    end
                end
            end
            %retorna a medi��o para o sistema de coordenadas do rob�
            % O ROB� � ORIENTADO PARA O EIXO X.
            Ps_i(1,:) = obj.s_i(1,:) - obj.Pos(1);
            Ps_i(2,:) = obj.s_i(2,:) - obj.Pos(2);
            obj.s = Ri\Ps_i;
            %adiciona ru�do na medi��o (a resolu��o da medi��o depende da resolu��o do
            %mapa)
            obj.s = obj.s + obj.ruido*randn(2,length(obj.angs));
            % sensor com ruido de medi��o no S.C. do ambiente
            aux = Ri*obj.s;
            obj.s2(1,:) = aux(1,:) + obj.Pos(1);
            obj.s2(2,:) = aux(2,:) + obj.Pos(2);

            % tratamento dos sensores (antiga fun��o FIM)

            obj.v_sensor = sqrt( (obj.s2(1,:)-pos_sensor(1,:)).^2 + (obj.s2(2,:)-pos_sensor(2,:)).^2 ); 
            v_colidiu = sqrt( (obj.s_i(1,:)-pos_sensor(1,:)).^2 + (obj.s_i(2,:)-pos_sensor(2,:)).^2 ); % dist�ncia real entre o rob� e os obstaculos
            ds0 = sort(v_colidiu);  % ds0 � a dist�ncia do sensor sem ru�do com a menor leitura
            if ds0(1) <= 1 %colis�o de pior caso com 
                obj.colidiu = 1;
            end
            
        end
        
        function obj = configuracao_inicial(obj,experimento,tempo_max)     
            
            obj.colidiu = 0; % verifica se o rob? colidiu para finalizar a simula??o
            tamos = experimento.tamos; % tempo de amostragem da simula??o em segundos
            % inicializa??o da posi??o do rob?
            x = experimento.rbx(end);
            y = experimento.rby(end);
            theta = experimento.ang(end)*pi/180; % inicia orientado para o eixo x do plano
            obj.Pos = [x ; y ; theta]; % ? a posi??o atual do rob? (aqui ? a posi??o inicial).
            obj.hPos = ones(3,500).*obj.Pos;
            
            % vetores sobre o experimento
            obj.plotInfo.P = zeros(3,round(tempo_max/tamos)-1);
            obj.plotInfo.P(:,1) = obj.Pos;  
            obj.plotInfo.Pvel = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pvel_medido = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pu = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pu_real = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pfi = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pfi_real = zeros(2,round(tempo_max/tamos)-1);
            obj.X = [0 ; 0]; % estado do sistema para simula??o din?mica
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % ?NGULOS NO SISTEMA DE COORDENADAS DO ROB?! -> 0 graus ? o eixo X (eixo de movimenta??o)
            resang = 5;
            obj.angs = [0:resang:360-resang]*pi/180;

            % inicializa??o dos sensores com zero;
            obj.s = zeros(2,length(obj.angs));
            obj.s_i = obj.s;
            obj.s2 = obj.s; %sensor com ru?do de medi??o no SC do ambiente
            
            
            
        end
        
        function obj = plotConfig(obj,re)
            
            % [vlimitex,vlimitey] = scircle2(0,0,1,0); % circulo limite de vis?o MATLAB
            aux = linspace(0,2*pi*100,100);
            vlimitex = cos(aux)';
            vlimitey = sin(aux)';
             % plot do rob� em terceira pessoa
            xc = vlimitex.*obj.raio;
            yc = vlimitey.*obj.raio;

            pxyc = [cos(obj.Pos(3)) -sin(obj.Pos(3)) ; sin(obj.Pos(3)) cos(obj.Pos(3))]*[xc' ; yc'];
            xc3 = pxyc(1,:)+obj.Pos(1);
            yc3 = pxyc(2,:)+obj.Pos(2);
            obj.plt.a = plot(xc3,yc3,'k');
            obj.plt.b = plot([obj.Pos(1) obj.Pos(1)+obj.raio*cos(obj.Pos(3))],[obj.Pos(2) obj.Pos(2)+obj.raio*sin(obj.Pos(3))],'r');
            obj.plt.c = plot(obj.hPos(1,:),obj.hPos(2,:),'k');
            
            obj.plt.d = plot(obj.path(:,1),obj.path(:,2),'--k');
            
            % Transpar�ncia desejada (0 = totalmente transparente, 1 = totalmente opaco)
            transparency = 0.25;  % Altere conforme necess�rio
            fillColor = [0.2, 0.6, 0.8];  % Cor azul claro, voc� pode ajustar isso
            xc = vlimitex.*re;
            yc = vlimitey.*re;
            pxyc = [cos(obj.Pos(3)) -sin(obj.Pos(3)) ; sin(obj.Pos(3)) cos(obj.Pos(3))]*[xc' ; yc'];
            xc4 = pxyc(1,:)+obj.Pos(1);
            yc4 = pxyc(2,:)+obj.Pos(2);
            obj.plt.e = patch(xc4, yc4, fillColor, 'EdgeColor', 'none', 'FaceAlpha', transparency);
%             obj.plt.e = plot(obj.e_path(1,:),obj.e_path(2,:),'b');

        end
        
        function obj = vecCorrecao(obj,iteracao)
            obj.plotInfo.P = obj.plotInfo.P(:,1:iteracao);
            obj.plotInfo.Pvel = obj.plotInfo.Pvel(:,1:iteracao);
            obj.plotInfo.Pvel_medido = obj.plotInfo.Pvel_medido(:,1:iteracao);
            obj.plotInfo.Pu = obj.plotInfo.Pu(:,1:iteracao);
            obj.plotInfo.Pu_real = obj.plotInfo.Pu_real(:,1:iteracao);
            obj.plotInfo.Pfi = obj.plotInfo.Pfi(:,1:iteracao);
            obj.plotInfo.Pfi_real = obj.plotInfo.Pfi_real(:,1:iteracao);
        end
    end
end