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
        angParaEscoltado  % No sistema de coordenadas do Robô
        sensorObstaculo
        colidiu
    % Variáveis do Controlador
        X_c % [l_e fi_e th_e]
        constantes_controle
        Ksi_r
        l_d
        l_desejado
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
        
            obj.colors = ['c','m','y','b','g','r'];
        end
        
        function obj = controle_e_navegacao(obj,iteracao,tamos,tempo,l_d,fi_d,escoltado)
            
%             th_e = escoltado.Pos(3);
%             obj.Pdes = escoltado.Pos(1:2) + l_d.*[cos(-fi_d+th_e);sin(-fi_d+th_e)];
%             
%             l_ei = norm(escoltado.Pos(1:2)-obj.Pos(1:2));
%             fi = atan2(obj.Pos(2)-escoltado.Pos(2),obj.Pos(1)-escoltado.Pos(1));
%             fi_ei = -fi+th_e;
%             
%             d = norm(obj.Pdes - obj.Pos(1:2));
%             theta_d = atan2(obj.Pdes(2) - obj.Pos(2),obj.Pdes(1) - obj.Pos(1)); % �ngulo de destino de -pi a pi
%             theta_e = theta_d - obj.Pos(3);
%             % converte theta_e para -pi a pi
%             if theta_e > pi, theta_e = theta_e - 2*pi; end
%             if theta_e < -pi, theta_e = theta_e + 2*pi; end
%             kw = 0.43;k1 = 1;
%             W = k1*obj.Wmax*tanh(kw*theta_e); 
%             if d < 5, W = 0; end
%             % converte theta_e para -pi/2 a pi/2
%             if theta_e > pi/2, theta_e = pi/2; end
%             if theta_e < -pi/2, theta_e = -pi/2; end
%             kv = 0.25;k2 = 1;
%             V = k2*obj.Vmax*tanh(kv*d)*cos(theta_e); 
             
            v_e = escoltado.plotInfo.Pvel_medido(1,iteracao);
            w_e = escoltado.plotInfo.Pvel_medido(2,iteracao);
            v_i = obj.plotInfo.Pvel_medido(1,iteracao);
            w_i = obj.plotInfo.Pvel_medido(2,iteracao);
             
            th_i = obj.Pos(3);
            th_e = escoltado.Pos(3);                

            if iteracao == 1
                 obj.X_c(1) = norm(escoltado.Pos(1:2)-obj.Pos(1:2));
                 obj.X_c(2) = atan2(obj.Pos(2)-escoltado.Pos(2),obj.Pos(1)-escoltado.Pos(1))-th_e;          
                 obj.X_c(3) = th_e - th_i; 
            end

            l_ei = obj.X_c(1);
            fi_ei = obj.X_c(2);
            gamma_ei = fi_ei + obj.X_c(3);
%             l_ei = norm(escoltado.Pos(1:2)-obj.Pos(1:2));
%             fi_ei = -atan2(obj.Pos(2)-escoltado.Pos(2),obj.Pos(1)-escoltado.Pos(1))+th_e;          
%             gamma_ei = (th_e + fi_ei) - th_i; 
            d = obj.L;
            
            dX_c = [v_i*cos(gamma_ei)-v_e*cos(fi_ei)+d*w_i*sin(gamma_ei),
                   (v_e*sin(fi_ei)-v_i*sin(gamma_ei)+d*w_i*cos(gamma_ei)-l_ei*w_e)/l_ei,
                   (w_e-w_i)];
            obj.X_c = obj.X_c + dX_c*tamos;
            
            l_ei = obj.X_c(1);
            fi_ei = obj.X_c(2);
            gamma_ei = fi_ei + obj.X_c(3);
           
            
            k1 = obj.constantes_controle(1);k2 = obj.constantes_controle(2);
            p_ei = (k1*tanh(l_d-l_ei)+v_e*cos(fi_ei))*sec(gamma_ei); 
            q_ei = (k2*l_ei*tanh(fi_d-fi_ei)-v_e*sin(fi_ei)+l_ei*w_e+p_ei*sin(gamma_ei));
            
            V = p_ei-d*w_i*tan(gamma_ei);
            W = cos(gamma_ei)*(q_ei/d);
            
            %%% Modificação da distância
            Rs = 30;
            Ra = 100;
            lobst = min(obj.v_sensor(obj.sensorObstaculo));
            A = (obj.l_desejado + Rs)/2;
            B = (obj.l_desejado - Rs)/2;
            C = (Ra + Rs)/2;
            k = obj.constantes_controle(3);
            obj.l_d = A + B*tanh(k*(lobst-C));
%             display(obj.l_d);
             
            obj.Pdes = escoltado.Pos(1:2) + l_d.*[cos(-fi_d+th_e);sin(-fi_d+th_e)];
            obj.Pdes_2 = escoltado.Pos(1:2) + l_ei.*[cos(-fi_ei+th_e);sin(-fi_ei+th_e)];


            %%% Velocidade Linear e Angular
%             Vmax = obj.Vmax; %m�ximo � aproximadamente 34 cm/s
%             Wmax = obj.Wmax; %m�ximo � aproximadamente 6 rad/s
            %%% Saturação Velocidade Linear e Angular
            if V > obj.Vmax
                V = obj.Vmax;
            end
            if W > obj.Wmax
                W = obj.Wmax;
            end     
            %%% Para o caso de usar apenas o modelo cinemático
            obj.Ksi_r = [V ; 0 ; W];
            
            %%% saída da função de controle deve ser U = [U_d ; U_e] entrada
            FI = obj.Modcin\[V ; W]; %FI = [FI_d ; FI_e];
            obj.U = [obj.ganhoft].*FI;
            
            
            obj.Ud = obj.U;
            % Saturação do sinal de controle
            if abs(obj.U(1)) > 1
                obj.U(1) = sign(obj.U(1));
            end
            if abs(obj.U(2)) > 1
                obj.U(2) = sign(obj.U(2));
            end
            
            % Calculo real
            l_d = norm(escoltado.Pos(1:2)-obj.Pos(1:2));
            fi_d = atan2(obj.Pos(2)-escoltado.Pos(2),obj.Pos(1)-escoltado.Pos(1))-th_e;  
            
            % informações para o plot
            obj.plotInfo.l_ei = [obj.plotInfo.l_ei l_ei];
            obj.plotInfo.l_d  = [obj.plotInfo.l_d l_d];
            obj.plotInfo.fi_ei = [obj.plotInfo.fi_ei fi_ei];
            obj.plotInfo.fi_d  = [obj.plotInfo.fi_d fi_d];
%             obj.plotInfo.gamma_ei = [obj.plotInfo.gamma_ei gamma_ei];
%             obj.plotInfo.p_ei = [obj.plotInfo.p_ei p_ei];
%             obj.plotInfo.q_ei = [obj.plotInfo.q_ei q_ei];
            obj.plotInfo.tempos  = [obj.plotInfo.tempos tempo];
            
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
                R = [cos(obj.Pos(3)) sin(obj.Pos(3)) 0 ; -sin(obj.Pos(3)) cos(obj.Pos(3)) 0 ; 0 0 1]; % matriz de rota��o
                [Ksir_real, X] = dinamica_zuadento(obj,tamos); %retorna as velocidades V e W reais no SC do rob�
                obj.X = X;
                Ksi_I = R\Ksir_real; % coloca as velocidades V e W reais no SC do ambiente
                obj.Pos = obj.Pos + Ksi_I*tamos; % atualiza��o da posi��o do rob� (integra��o no SC do ambiente)
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

                
                obj.plotInfo.Pvel(:,iteracao+1) = Ksir_d; % atualiza o vetor das velocidades (comandos) do rob� durante o experimento.
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob� durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi��es do rob� durante o experimento (SC do ambiente).
                obj.plotInfo.Pu(:,iteracao+1) = obj.Ud; % U = [U_d ; U_e] de -1 a 1 % valor desejado
                obj.plotInfo.Pu_real(:,iteracao+1) = Ureal; % U = [U_d ; U_e] de -1 a 1 % valor real
                obj.plotInfo.Pfi(:,iteracao+1) = fi; % U = [Fi_d ; Fi_e] % valor desejado de fi
                obj.plotInfo.Pfi_real(:,iteracao+1) = FIreal; % U = [Fi_d ; Fi_e] % valor real de fi
                
        end
        
        function obj = simulacao_apenas_cinematica(obj,tamos,iteracao)
            
                R = [cos(obj.Pos(3)) sin(obj.Pos(3)) 0 ; -sin(obj.Pos(3)) cos(obj.Pos(3)) 0 ; 0 0 1]; % matriz de rota��o
                Ksi_I = R\obj.Ksi_r; % coloca as velocidades V e W reais no SC do ambiente
                obj.Pos = obj.Pos + Ksi_I*tamos; % atualiza��o da posi��o do rob� (integra��o no SC do ambiente)
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = obj.Ksi_r(1);
                Wmedido = obj.Ksi_r(3);
                
                
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob� durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi��es do rob� durante o experimento (SC do ambiente).
              
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

            obj.angParaEscoltado = [];
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
                                obj.angParaEscoltado = [obj.angParaEscoltado obj.angs(k)];
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
        
        function obj = configuracao_inicial(obj,experimento,tempo_max,k)     
            
            obj.colidiu = 0; % verifica se o rob� colidiu para finalizar a simula��o
            tamos = experimento.tamos; % tempo de amostragem da simula��o em segundos
            % inicializa��o da posi��o do rob�
            x = experimento.rbx(k);
            y = experimento.rby(k);
            theta = experimento.ang(k)*pi/180; % inicia orientado para o eixo x do plano
            obj.Pos = [x ; y ; theta]; % � a posi��o atual do rob� (aqui � a posi��o inicial).
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
            obj.plotInfo.l_d  = [];
            obj.plotInfo.fi_ei = [];
            obj.plotInfo.fi_d  = [];
            obj.plotInfo.p_ei = [];
            obj.plotInfo.q_ei = [];
            obj.plotInfo.gamma_ei  = [];
            obj.plotInfo.tempos  = [];
            obj.X = [0 ; 0]; % estado do sistema para simula��o din�mica
            obj.X_c = [experimento.l_ei(k) ; experimento.fi_ei(k) ; experimento.theta_ei(k)]; % estado do controlador
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % �NGULOS NO SISTEMA DE COORDENADAS DO ROB�! -> 0 graus � o eixo X (eixo de movimenta��o)
            resang = 5;
            obj.angs = [0:resang:360-resang]*pi/180;

            % inicializa��o dos sensores com zero;
            obj.s = zeros(2,length(obj.angs));
            obj.s_i = obj.s;
            obj.s2 = obj.s; %sensor com ru�do de medi��o no SC do ambiente
            
            
            
        end
        
        function obj = plotConfig(obj,k)
            
            % [vlimitex,vlimitey] = scircle2(0,0,1,0); % circulo limite de vis�o MATLAB
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
            obj.plt.c = plot(obj.Pdes(1),obj.Pdes(2),'.','MarkerEdgeColor',obj.colors(2),'MarkerSize',7);
            obj.plt.d = plot(obj.Pdes_2(1),obj.Pdes_2(2),'.','MarkerEdgeColor',obj.colors(5),'MarkerSize',7);

            if k<=3
                obj.plt.e = plot(obj.hPos(1,:),obj.hPos(2,:),obj.colors(3+k));
            else
                obj.plt.e = plot(obj.hPos(1,:),obj.hPos(2,:),obj.colors(4));
            end

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
        
        
    end
end

