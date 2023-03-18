%             if 0
                obj.posEscoltado_ = [obj.posEscoltado_ posEscoltado(1:2)]; 
                obj.orientacaoEscoltado = posEscoltado(3);
%             else
%                 obj.posEscoltado_ = [obj.posEscoltado_ escoltado.Pos(1:2)];
%                 obj.orientacaoEscoltado = escoltado.Pos(3);
%             end
%             n = round((iteracao));
%             if length(obj.posEscoltado) > 5, n = 5; end
            if (iteracao==1)
                dpos = [0;0];
            else
                dpos = obj.posEscoltado_(:,end)-obj.posEscoltado_(:,end-1);                
            end
            
%             obj.orientacaoEscoltado = escoltado.Pos(3);

            d = escoltado.L;
            phi_e = obj.orientacaoEscoltado;
            R = [cos(phi_e) sin(phi_e); -sin(phi_e) cos(phi_e)];
            u = dpos/(tamos);
            VW = [1 0;0 1/d]*(R*u);

            v_e = VW(1);       
            w_e = VW(2);
            if abs(v_e) > obj.Vmax
                v_e = sign(v_e)*obj.Vmax;
            end
            if abs(w_e) > obj.Wmax
                w_e = sign(w_e)*obj.Wmax;
            end  

            
            v_i = obj.plotInfo.Pvel_medido(1,iteracao);
            w_i = obj.plotInfo.Pvel_medido(2,iteracao);
            
%             Pe = [obj.posEscoltado(:,end);obj.orientacaoEscoltado];
            Pe = obj.posEscoltado_(:,end);

            th_i = obj.Pos(3);
            th_e = obj.orientacaoEscoltado;      
            

            obj.X_c(1) = norm(Pe(1:2)-obj.Pos(1:2));
            obj.X_c(2) = atan2(obj.Pos(2)-Pe(2),obj.Pos(1)-Pe(1))-th_i;
%             obj.X_c(2) = obj.angParaEscoltado(1)-pi;
            obj.X_c(3) = th_e - th_i; 

            l_ei = obj.X_c(1);
            fi_ei = obj.X_c(2);
            gamma_ei = fi_ei + obj.X_c(3);

            d = obj.L;
             
            
            k1 = obj.constantes_controle(1);k2 = obj.constantes_controle(2);
            p_ei = (k1*tanh(l_d-l_ei)+v_e*cos(fi_ei))*sec(gamma_ei); 
            q_ei = (k2*l_ei*tanh(fi_d-fi_ei)-v_e*sin(fi_ei)+l_ei*w_e+p_ei*sin(gamma_ei));
            
            V = p_ei-d*w_i*tan(gamma_ei);
            W = cos(gamma_ei)*(q_ei/d);
            
            %%% Modificação da distância
            Rs = 30;
            Ra = 100;
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
%             display(obj.l_d);
             
            obj.Pdes = Pe(1:2) + l_d.*[cos(-fi_d+th_e);sin(-fi_d+th_e)];
%             obj.Pdes = Pe(1:2);
            obj.Pdes_2 = posEscoltado;
%             obj.Pdes_2 = Pe(1:2) + l_ei.*[cos(-fi_ei+th_e);sin(-fi_ei+th_e)];
%             obj.Pdes_2 = obj.PosEscoltado;



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
            obj.plotInfo.fi_ei = [obj.plotInfo.fi_ei (fi_ei-fi_d)];
%             obj.plotInfo.fi_d  = [obj.plotInfo.fi_d abs(fi_ei-fi_d)];
%             obj.plotInfo.gamma_ei = [obj.plotInfo.gamma_ei gamma_ei];
%             obj.plotInfo.p_ei = [obj.plotInfo.p_ei p_ei];
%             obj.plotInfo.q_ei = [obj.plotInfo.q_ei q_ei];
            obj.plotInfo.tempos  = [obj.plotInfo.tempos tempo];