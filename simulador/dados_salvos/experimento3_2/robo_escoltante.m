classdef robo_escoltante
    % robo classe que representa o robô
    
    properties
    % Características físicas do robô    
        r  % 5.3000
        L  % 5.6250
        Modelo  % [2×2 ss]
        raio  % 8
        Modcin  % [2×2 double]
        ganhoft  % [2×1 double]
        Vmax  % 34.1570
        Wmax  % 6.0724
        ruido  %  1
        saturacao  % 200
        Rs
        Ra
        Rc
    % Informações sensoriais do robô
        Pos 
        hPos
        Pdes
        Pdes_2
        v_sensor
        s_i
        s
        s2 
        angs
        posEscoltado
        posEscoltado_
        orientacaoEscoltado
        sensorEscoltado  % No sistema de coordenadas do Robô
        sensorObstaculo
        sensorRobos
        colidiu
        falhou
    % Variáveis do Controlador
        X_c % [l_e fi_e th_e]
        constantes_controle
        Ksi_r
        l_d
        l_desejado
        l_obst
    % Das especificações
        r_p
    % Variáveis Controlador baixo nível PI
        err_i  % integral do erro
    % Variáveis Controle desvio de obstáculo 
        Freal  % memória
    % Sinais de Controle
        U    % Contem o sinal de controle da roda esquerda e direita
        Ud   % Sinal de controle desejado
        X    % estado dinâmico do sistema
    % Informações para o plot
        plotInfo
        plt
        colors
    end
    
    methods
        function obj = robo_escoltante()
            %%% Parâmetros de configuração das características do robô.
            % Wd/FId = A/(s+B); %velocidade angula na saída em rad/s
            %    -15.66
            %   --------
            %   s + 4.87
            %    -15.11
            %   ---------
            %   s + 5.014
            % We/FIe = C/(s+D); %velocidade angula na saída em rad/s
            %    17.75
            %   ---------
            %   s + 5.833
            %    13.11
            %   ---------
            %   s + 4.286
            obj.r = 5.3; %raio da roda
            obj.L = 11.25/2; %distância da roda ao centro do robô (metade da distancia entre as rodas)
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
            % Y = [V ; W] saídas

            obj.raio = 8; %raio do robô para verificar colisão e plotar o robô [cm]
            obj.Modcin = Mc; %modelo cinemático direto
            obj.ganhoft = [B/A_ ; D/C_];
            % U = inv(obj.Modcin)*[V ; W];

            fi = [1 ; 1]./[obj.ganhoft];
            aux = obj.Modcin*(fi);
            obj.Vmax = aux(1); %cálculo da vellcidade linear máxima a partir do modelo identificado
            fi = [1 ; -1]./[obj.ganhoft];
            aux = obj.Modcin*(fi);
            obj.Wmax = aux(2); %cálculo da vellcidade angular máxima a partir do modelo identificado
            
            obj.Rs = 30;
            obj.Ra = 120;
            obj.Rc = 200;
            obj.l_obst = 120;
            obj.r_p = 120;
            
            obj.err_i = [0;0];
            obj.Freal = [0;0];
            
            obj.colors = ['b','g','r','c','m','y','b','g','r'];
        end
        
        function obj = controle_e_navegacao(obj,iteracao,tamos,tempo,l_d,fi_d,escoltado,habilitaDinamica)
            
%             Desai_estimacaoXY_ideal
            Desai_PosEscoltadoConhecida
%             Desai_estimacaoXY_Triang01
            
                        
        end
        
        function obj = TriangToTal(obj,dados)
            
            x1 = dados.x(1) - dados.x(2); y1 = dados.y(1) - dados.y(2);
            x3 = dados.x(3) - dados.x(2); y3 = dados.y(3) - dados.y(2);
            
            T12 = cot(dados.phi(2)-dados.phi(1)); T23 = cot(dados.phi(3)-dados.phi(2));
            T31 = (1+T12*T23)/(T12+T23);
            
            x12 = x1 + T12*y1; y12 = y1 - T12*x1;
            x23 = x3 - T23*y3; y23 = y3 + T23*x3;
            x31 = (x1 + x3) + T31*(y3-y1); y31 = (y1 + y3) - T31*(x3-x1);
            
            k31 = x1*x3 + y1*y3 + T31*(x1*y3-x3*y1);
            
            D = (x12-x23)*(y23-y31) - (y12-y23)*(x23-x31);
            
            obj.posEscoltado(1:2) = [dados.x(2);dados.y(2)]...
                                    + (k31/D)*[(y12-y23) ; (x23-x12) ];
            
            obj.posEscoltado(3) = atan2(dados.y(1)-obj.posEscoltado(2),dados.x(1)-obj.posEscoltado(1))-dados.phi(1);
            
        end
             
        function [pos,psi] = envia_pacote(obj,escoltado)
%             R = [cos(obj.Pos(3)) sin(obj.Pos(3)) 0 ; -sin(obj.Pos(3)) cos(obj.Pos(3)) 0 ; 0 0 1];
%             pvel = R\obj.Ksi_r;
            pos = obj.Pos';
            psi = atan2(obj.Pos(2)-escoltado.Pos(2),obj.Pos(1)-escoltado.Pos(1))-obj.Pos(3);
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
            
            if ~obj.falhou
                
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

                fi = obj.Modcin\[obj.Ksi_r(1) ; obj.Ksi_r(3)]; %FI = [FI_d ; FI_e];
                Ud = [obj.ganhoft].*fi;
                Ksir_d = [obj.Ksi_r(1) ; obj.Ksi_r(3)];

%                 fi = obj.Ud./[obj.ganhoft]; % fi desejado 
%                 Ksir_d = obj.Modcin*(fi); % converte para comandos de velocidade (apenas para registro e plots futuros)
                
                obj.plotInfo.Pvel(:,iteracao+1) = Ksir_d; % atualiza o vetor das velocidades (comandos) do rob? durante o experimento.
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
%                 obj.plotInfo.Pu(:,iteracao+1) = obj.Ud; % U = [U_d ; U_e] de -1 a 1 % valor desejado
                obj.plotInfo.Pu(:,iteracao+1) = Ud; % U = [U_d ; U_e] de -1 a 1 % valor desejado         
                obj.plotInfo.Pu_real(:,iteracao+1) = Ureal; % U = [U_d ; U_e] de -1 a 1 % valor real
                obj.plotInfo.Pfi(:,iteracao+1) = fi; % U = [Fi_d ; Fi_e] % valor desejado de fi
                obj.plotInfo.Pfi_real(:,iteracao+1) = FIreal; % U = [Fi_d ; Fi_e] % valor real de fi
                
            else
                
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = 0;
                Wmedido = 0;

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
                
        end
        
        function obj = simulacao_apenas_cinematica(obj,tamos,iteracao)
            
            if ~obj.falhou
                R = [cos(obj.Pos(3)) sin(obj.Pos(3)) 0 ; -sin(obj.Pos(3)) cos(obj.Pos(3)) 0 ; 0 0 1]; % matriz de rota??o
                Ksi_I = R\obj.Ksi_r; % coloca as velocidades V e W reais no SC do ambiente
                obj.Pos = obj.Pos + Ksi_I*tamos; % atualiza??o da posi??o do rob? (integra??o no SC do ambiente)
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = obj.Ksi_r(1);
                Wmedido = obj.Ksi_r(3);
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
            else
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = 0;
                Wmedido = 0;
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
            end
        end
        
        function obj = simulacao_sensores(obj,A)
           
            %%% tratamento dos sensores (antiga função inicio)
            Ps_i = obj.s_i;
            [ymaxA , xmaxA] = size(A);
            smax = obj.saturacao*(ones(1,length(obj.angs)));
            smax = smax + obj.raio;
            %s é um vetor cujos elementos representam o ponto da leitura máxima (a priori) para
            %cada um dos sensores no sistema de coordenadas do robô
            obj.s = [smax.*cos(obj.angs) ; smax.*sin(obj.angs)]; % vetor no SC do robô (ainda saturado)    
            Ri = [cos(obj.Pos(3)) -sin(obj.Pos(3)); sin(obj.Pos(3)) cos(obj.Pos(3))]; % matriz de rotação
            %s_i é o vetor s colocado no sistema de coordenadas do ambiente (sem ruído) (ainda saturado)
            obj.s_i = Ri*obj.s;
            obj.s_i(1,:) = obj.s_i(1,:) + obj.Pos(1);
            obj.s_i(2,:) = obj.s_i(2,:) + obj.Pos(2);

            pos_sensor = [obj.raio.*cos(obj.angs) ; obj.raio.*sin(obj.angs)];
            pos_sensor = Ri*pos_sensor;
            pos_sensor(1,:) = pos_sensor(1,:) + obj.Pos(1);
            pos_sensor(2,:) = pos_sensor(2,:) + obj.Pos(2);

            obj.sensorEscoltado = [];
            obj.sensorRobos = [];
            obj.sensorObstaculo = [];
            for k=1:length(obj.angs)
                Vsx = round(linspace(pos_sensor(1,k),obj.s_i(1,k),obj.saturacao));
                Vsy = round(linspace(pos_sensor(2,k),obj.s_i(2,k),obj.saturacao));
                Vs = [Vsx ; Vsy];
                for j=1:obj.saturacao
                    if (Vs(1,j)>0 && Vs(1,j)<=xmaxA && Vs(2,j)>0 && Vs(2,j)<=ymaxA) % se a leitura esta dentro do mapa
                        if A(Vs(2,j),Vs(1,j)) ~= 255
                            obj.s_i(1,k) = Vs(1,j);
                            obj.s_i(2,k) = Vs(2,j);
                            if A(Vs(2,j),Vs(1,j)) == 100
                                obj.sensorEscoltado = [obj.sensorEscoltado k];
                            end
                            if A(Vs(2,j),Vs(1,j)) == 50
                                obj.sensorRobos = [obj.sensorRobos k];
                            end
                            if A(Vs(2,j),Vs(1,j)) == 0
                                obj.sensorObstaculo = [obj.sensorObstaculo k];
                            end
                            break;
                        end
                    end
                end
            end
            %retorna a medição para o sistema de coordenadas do robô
            % O ROBÔ É ORIENTADO PARA O EIXO X.
            Ps_i(1,:) = obj.s_i(1,:) - obj.Pos(1);
            Ps_i(2,:) = obj.s_i(2,:) - obj.Pos(2);
            obj.s = Ri\Ps_i;
            %adiciona ruído na medição (a resolução da medição depende da resolução do
            %mapa)
            obj.s = obj.s + obj.ruido*randn(2,length(obj.angs));
            % sensor com ruido de medição no S.C. do ambiente
            aux = Ri*obj.s;
            obj.s2(1,:) = aux(1,:) + obj.Pos(1);
            obj.s2(2,:) = aux(2,:) + obj.Pos(2);

            % tratamento dos sensores (antiga função FIM)

            obj.v_sensor = sqrt( (obj.s2(1,:)-pos_sensor(1,:)).^2 + (obj.s2(2,:)-pos_sensor(2,:)).^2 ); 
            v_colidiu = sqrt( (obj.s_i(1,:)-pos_sensor(1,:)).^2 + (obj.s_i(2,:)-pos_sensor(2,:)).^2 ); % distância real entre o robô e os obstaculos
            ds0 = sort(v_colidiu);  % ds0 é a distância do sensor sem ruído com a menor leitura
            if ds0(1) <= 1 %colisão de pior caso com 
                obj.colidiu = 1;
            end
            
        end
        
        function obj = configuracao_inicial(obj,experimento,escoltado,tempo_max,k)     
            
            obj.colidiu = 0; % verifica se o rob? colidiu para finalizar a simula??o
            obj.falhou = 0;
            tamos = experimento.tamos; % tempo de amostragem da simula??o em segundos
            % inicializa??o da posi??o do rob?
            x = experimento.rbx(k);
            y = experimento.rby(k);
            theta = experimento.ang(k)*pi/180; % inicia orientado para o eixo x do plano
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
            obj.plotInfo.l_ei = [];
            obj.plotInfo.l = [];
            obj.plotInfo.r_p = [];
            obj.plotInfo.r_p_min = [];
%             obj.plotInfo.l_d  = [];
            obj.plotInfo.fi_ei = [];
            obj.plotInfo.fi = [];
%             obj.plotInfo.fi_d  = [];
            obj.plotInfo.p_ei = [];
            obj.plotInfo.q_ei = [];
            obj.plotInfo.gamma_ei  = [];
            obj.plotInfo.tempos  = [];
            obj.X = [0 ; 0]; % estado do sistema para simula??o din?mica
            obj.X_c = [experimento.l_ei(k) ; experimento.fi_ei(k) ; experimento.theta_ei(k)]; % estado do controlador
            obj.posEscoltado = escoltado.Pos;
            obj.posEscoltado_ = escoltado.Pos(1:2);
            obj.orientacaoEscoltado = escoltado.Pos(3);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % ?NGULOS NO SISTEMA DE COORDENADAS DO ROB?! -> 0 graus ? o eixo X (eixo de movimenta??o)
            resang = 5;
            obj.angs = [0:resang:360-resang]*pi/180;

            % inicializa??o dos sensores com zero;
            obj.s = zeros(2,length(obj.angs));
            obj.s_i = obj.s;
            obj.s2 = obj.s; %sensor com ru?do de medi??o no SC do ambiente          
        end
        
        function obj = plotConfig(obj,k)
            
            % [vlimitex,vlimitey] = scircle2(0,0,1,0); % circulo limite de vis?o MATLAB
            aux = linspace(0,2*pi*100,100);
            vlimitex = cos(aux)';
            vlimitey = sin(aux)';
             % plot do robô em terceira pessoa
            xc = vlimitex.*obj.raio;
            yc = vlimitey.*obj.raio;

            pxyc = [cos(obj.Pos(3)) -sin(obj.Pos(3)) ; sin(obj.Pos(3)) cos(obj.Pos(3))]*[xc' ; yc'];
            xc3 = pxyc(1,:)+obj.Pos(1);
            yc3 = pxyc(2,:)+obj.Pos(2);
            obj.plt.a = plot(xc3,yc3,'b');
            obj.plt.b = plot([obj.Pos(1) obj.Pos(1)+obj.raio*cos(obj.Pos(3))],[obj.Pos(2) obj.Pos(2)+obj.raio*sin(obj.Pos(3))],'r');
%             obj.plt.c = plot(obj.s2(1,:),obj.s2(2,:),'.','MarkerEdgeColor',obj.colors(2),'MarkerSize',7);
            obj.plt.c = plot(obj.Pdes(1),obj.Pdes(2),'.','MarkerEdgeColor','m','MarkerSize',7);
%             obj.plt.d = plot(obj.Pdes_2(1),obj.Pdes_2(2),'.','MarkerEdgeColor',obj.colors(5),'MarkerSize',7);

%             if k<=3
            obj.plt.e = plot(obj.hPos(1,:),obj.hPos(2,:),obj.colors(k));
%             else
%                 obj.plt.e = plot(obj.hPos(1,:),obj.hPos(2,:),obj.colors(4));
%             end
            transparency = 0.25;  % Altere conforme necessário
            fillColor = [0.8, 0.2, 0.2];  % Cor vermelho claro, você pode ajustar isso
            xc = vlimitex.*obj.r_p;
            yc = vlimitey.*obj.r_p;
            pxyc = [cos(obj.Pos(3)) -sin(obj.Pos(3)) ; sin(obj.Pos(3)) cos(obj.Pos(3))]*[xc' ; yc'];
            xc4 = pxyc(1,:)+obj.Pos(1);
            yc4 = pxyc(2,:)+obj.Pos(2);
            obj.plt.f = patch(xc4, yc4, fillColor, 'EdgeColor', 'none', 'FaceAlpha', transparency);


        end
        
        function obj = vecCorrecao(obj,iteracao, habilitaDinamica)
            if habilitaDinamica==1
                obj.plotInfo.P = obj.plotInfo.P(:,1:iteracao);
                obj.plotInfo.Pvel = obj.plotInfo.Pvel(:,1:iteracao);
                obj.plotInfo.Pvel_medido = obj.plotInfo.Pvel_medido(:,1:iteracao);
                obj.plotInfo.Pu = obj.plotInfo.Pu(:,1:iteracao);
                obj.plotInfo.Pu_real = obj.plotInfo.Pu_real(:,1:iteracao);
                obj.plotInfo.Pfi = obj.plotInfo.Pfi(:,1:iteracao);
                obj.plotInfo.Pfi_real = obj.plotInfo.Pfi_real(:,1:iteracao);
            else
                obj.plotInfo.P = obj.plotInfo.P(:,1:iteracao);
                obj.plotInfo.Pvel_medido = obj.plotInfo.Pvel_medido(:,1:iteracao);
            end
        end       
        
        function [V,W] = desviar_obstaculo(obj,dmax,gamma_ei,d)
            %% Controle Reativo - Forças Virtuais
%             % Modelo Cinemático:
%             dmin = 8; Fmax = 1;
%             s = 15;
            % Modelo Dinâmico:
            dmin = 10; Fmax = 20;
            s = 10;

            % Sensor do Lado esquerdo
            sensores = setdiff([1:72],obj.sensorEscoltado);
            [~,u] = find(obj.angs(sensores)<=pi);
            [dL, I] = min(obj.v_sensor(sensores(u)));
            alphaL = obj.angs(sensores(u(I)));
            cl = (dmax > dL);
            Fl = cl*Fmax*((dmax - dL)/(dmax-dmin)).^s;

            % Sensor do Lado direito
            [~,u] = find(obj.angs(sensores)>pi);
            [dR, I] = min(obj.v_sensor(sensores(u)));
            alphaR = obj.angs(sensores(u(I)));
            cr = (dmax > dR);
            Fr = cr*Fmax*((dmax - dR)/(dmax-dmin)).^s;            

            % Seja a Admitância mecânica (Y):
            % Y = A/(s+B)
            % Y = Freal/F => (Freal(k+1) - Freal)/dt = -BFreal + AF
            %             =>  Freal(k+1) = (1-dtB)Freal + dtAF
            B = 2; A = (B*(obj.Wmax/Fmax)); dt = 0.01;
            F = [Fr;Fl];
            F = (1-dt*B)*obj.Freal + dt*A*F;
            obj.Freal = F;
        %             % Admitância mecânica (Y) considerando apenas a constante elástica
        %             Y = obj.Wmax;  % baseado na correção de pior caso          

            W =  (F(1)-F(2));
            V =  -d*W*tan(gamma_ei);

            % Debug do Controle de desvio de obstáculo
            if (dL<dmax)|(dR<dmax)
                    display('desviar de obstáculo');
            end

        end
        
        function [V,W] = desviar_obstaculo2(obj,Pdes)
            %% Controle Reativo - Campos Potênciais
            ka = 1;
            grad_Ua = -ka*(obj.Pos(1:2)-Pdes);
            

            kr = 1e3; 
            ro_L = (obj.Rs-obj.raio); [ro,I]=min(obj.v_sensor);
            grad_Ur = kr*((1/ro - 1/ro_L)*(1/(ro^3)))*(obj.Pos(1:2)-obj.s2(:,I))*(ro<=ro_L);

            
            grad_U = grad_Ua + grad_Ur; % + grad_Ur2 + grad_Ur3;


            %% Norma do vetor gradiente (d)
            d = norm(grad_U); %dist?ncia at? o destino

            %% angulação do vetor gradiente (theta_e)
            theta_d = atan2(grad_U(2),grad_U(1)); % ?ngulo de destino de -pi a pi
            theta_e = mod(theta_d - obj.Pos(3)+pi,2*pi)-pi;

%             % converte theta_e para -pi a pi
%             if theta_e > pi/2, theta_e = pi/2; end
%             if theta_e < -pi/2, theta_e = -pi/2; end

            %% C?culo das velocidades linear (V) e angular (W) do rob? (controle de posi??o final simples)
            Vmax = obj.Vmax; Wmax = obj.Wmax;
            kv = 0.03;
            V = Vmax*tanh(kv*d)*cos(theta_e);
            kw = 0.6;
            W = Wmax*tanh(kw*theta_e);
        end
        
        function obj = simulacao_falha(obj)
                obj.falhou = 1;
        end
    end
end