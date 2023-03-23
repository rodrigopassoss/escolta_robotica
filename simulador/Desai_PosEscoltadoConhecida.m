
            v_e = escoltado.plotInfo.Pvel_medido(1,iteracao);
            w_e = escoltado.plotInfo.Pvel_medido(2,iteracao);

            
            v_i = obj.plotInfo.Pvel_medido(1,iteracao);
            w_i = obj.plotInfo.Pvel_medido(2,iteracao);
            
%             Pe = [obj.posEscoltado(:,end);obj.orientacaoEscoltado];
            Pe = escoltado.Pos;

            th_i = obj.Pos(3);
            th_e = escoltado.Pos(3);      
            

            d = obj.L;
            Pos = obj.Pos(1:2)  + d*[cos(obj.Pos(3));sin(obj.Pos(3))];
            
            obj.X_c(1) = norm(Pe(1:2)-Pos(1:2));
            
            obj.X_c(2) = mod(atan2(Pos(2)-Pe(2),Pos(1)-Pe(1))-th_e + pi, 2*pi)-pi;
            obj.X_c(3) = mod(th_e - th_i + pi, 2*pi)-pi; 

            l_ei = obj.X_c(1);
            fi_ei = obj.X_c(2);
            gamma_ei = mod(fi_ei + obj.X_c(3) + pi, 2*pi)-pi;   
%             gamma_ei = fi_ei + obj.X_c(3);             

            
            k1 = obj.constantes_controle(1);k2 = obj.constantes_controle(2);
            p_ei = (k1*tanh(l_d-l_ei)+v_e*cos(fi_ei))*sec(gamma_ei); 
            ang_err =  mod(fi_d - fi_ei + pi, 2*pi) -pi;
            q_ei = (k2*l_ei*tanh(ang_err)-v_e*sin(fi_ei)+l_ei*w_e+p_ei*sin(gamma_ei));
            
            V = p_ei-d*w_i*tan(gamma_ei);
            W = cos(gamma_ei)*(q_ei/d);
            
            %%% Modificação da distância
            Rs = obj.Rs;
            Ra = obj.Ra;
            if length(obj.sensorObstaculo)~=0
                lobst = min(obj.v_sensor(obj.sensorObstaculo));
            else
                lobst = obj.v_sensor(1);
            end
                
            A = (obj.l_desejado + Rs)/2;
            B = (obj.l_desejado - Rs)/2;
            C = (Ra + Rs)/2;
            k = obj.constantes_controle(3);
            obj.l_d = A + B*tanh(k*(lobst-C));
            
            ang1 = fi_d+th_e;
            obj.Pdes = Pe(1:2) + l_d.*[cos(ang1);sin(ang1)];
            ang2 = fi_ei+th_e;
            obj.Pdes_2 = Pe(1:2) + l_ei.*[cos(ang2);sin(ang2)];

            %%% Velocidade Linear e Angular
            Vmax = obj.Vmax; %m�ximo � aproximadamente 34 cm/s
            Wmax = obj.Wmax; %m�ximo � aproximadamente 6 rad/s
            %% Saturação Velocidade Linear e Angular
            if abs(V) > obj.Vmax
                V = sign(V)*obj.Vmax;
            end
            if abs(W) > obj.Wmax
                W = sign(W)*obj.Wmax;
            end  
            
            % Controle de desvio de obstáculo
            d = min(obj.v_sensor);
            if d < obj.raio 
                [V,W] = obj.desviar_obstaculo(obj.Pdes);
            end
            
            %%% Para o caso de usar apenas o modelo cinemático
            obj.Ksi_r = [V ; 0 ; W];
            
            %%% saída da função de controle deve ser U = [U_d ; U_e] entrada
            FI = obj.Modcin\[V ; W]; %FI = [FI_d ; FI_e];
            obj.U = [obj.ganhoft].*FI;
            
            
            % Saturação do sinal de controle
            if abs(obj.U(1)) > 1
                obj.U(1) = sign(obj.U(1));
            end
            if abs(obj.U(2)) > 1
                obj.U(2) = sign(obj.U(2));
            end
            obj.Ud = obj.U;
            
            % Calculo real
%             l_d = norm(escoltado.Pos(1:2)-obj.Pos(1:2));
%             fi_d = atan2(obj.Pos(2)-escoltado.Pos(2),obj.Pos(1)-escoltado.Pos(1))-th_e;  
            
            % informações para o plot
            obj.plotInfo.l_ei = [obj.plotInfo.l_ei (l_ei-obj.l_d)];
%             obj.plotInfo.l_d  = [obj.plotInfo.l_d abs(l_ei-obj.l_d)];
            obj.plotInfo.fi_ei = [obj.plotInfo.fi_ei (mod(fi_ei-fi_d+pi,2*pi)-pi)];
%             obj.plotInfo.fi_d  = [obj.plotInfo.fi_d abs(fi_ei-fi_d)];
%             obj.plotInfo.gamma_ei = [obj.plotInfo.gamma_ei gamma_ei];
%             obj.plotInfo.p_ei = [obj.plotInfo.p_ei p_ei];
%             obj.plotInfo.q_ei = [obj.plotInfo.q_ei q_ei];
            obj.plotInfo.tempos  = [obj.plotInfo.tempos tempo];