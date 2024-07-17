%%          Atualização das Variáveis
            v_e = escoltado.plotInfo.Pvel_medido(1,iteracao);
            w_e = escoltado.plotInfo.Pvel_medido(2,iteracao);

            
            v_i = obj.plotInfo.Pvel_medido(1,iteracao);
            w_i = obj.plotInfo.Pvel_medido(2,iteracao);
            
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

%%          Cálculo das entradas de controle
            %Constantes de saturação
            l_sat = 20;
            fi_sat = 0.6;

            k1 = obj.constantes_controle(1); k2 = obj.constantes_controle(2);
%             ang_err =  mod(fi_d - fi_ei + pi, 2*pi) -pi;
            ang_err =  fi_sat*tanh((mod(fi_d - fi_ei + pi, 2*pi) -pi)/fi_sat);
            
%             p_ei = (k1*(l_d-l_ei)+v_e*cos(fi_ei))*sec(gamma_ei); 
            p_ei = (k1*l_sat*tanh((l_d-l_ei)/l_sat)+v_e*cos(fi_ei))*sec(gamma_ei);
            q_ei = (k2*l_ei*(ang_err)-v_e*sin(fi_ei)+l_ei*w_e+p_ei*sin(gamma_ei));            

            W = cos(gamma_ei)*(q_ei/d);
            V = p_ei-d*W*tan(gamma_ei);
%%          Módulo de desvio de obstáculo 
            dmax = 20;
            [Vc,Wc] = obj.desviar_obstaculo(dmax,gamma_ei,d); % desvio de obstáculo;
            W = W + Wc;
            V = V + Vc;
            %----------------------------- 
%%         Obtensão do sinal de controle
            %%% Velocidade Linear e Angular
            Vmax = obj.Vmax; %m?ximo ? aproximadamente 34 cm/s
            Wmax = obj.Wmax; %m?ximo ? aproximadamente 6 rad/s
            if abs(V) > Vmax
                V = sign(V)*Vmax;
            end
            if abs(W) > Wmax
                W = sign(W)*Wmax;
            end  
            %%% Para o caso de usar apenas o modelo cinemático
            obj.Ksi_r = [V ; 0 ; W];
            
            %%% saída da função de controle deve ser U = [U_d ; U_e] entrada
            FI = obj.Modcin\[V ; W]; %FI = [FI_d ; FI_e];
            %%% loop de controle de velocidade para o caso com a consideração dinâmica             
            if habilitaDinamica==1
                controle_velocidade
            else
                obj.U = [obj.ganhoft].*FI;
            end
%             %%% Saturação do sinal de controle
            if abs(obj.U(1)) > 1
                obj.U(1) = sign(obj.U(1));
            end
            if abs(obj.U(2)) > 1
                obj.U(2) = sign(obj.U(2));
            end
            obj.Ud = obj.U;
            
%%          Modificação da distância
            Rs = obj.Rs;
            Ra = obj.Ra;
            if length(obj.sensorObstaculo)~=0
                lobst = min(obj.v_sensor(obj.sensorObstaculo));
            else
                lobst = obj.v_sensor(1);
            end
            obj.l_obst = lobst;
            
            alpha = pi/obj.constantes_controle(4);
            Rs = obj.Rs; Ra = obj.Ra; %Rd = obj.l_desejado; 
            Rd = cot(alpha)*Ra;
            if Rd > Ra, Rd=Ra; end
            A = (Rd + Rs)/2;
            B = (Rd - Rs)/2;
            C = ((escoltado.Rs+Rd)*tan(alpha))/2; %(C = (Ra + Rs)/2) e Ra = Rd*tan(alpha)
            k = (B*sin(alpha))^-1;
%             k = obj.constantes_controle(3);
%             alpha = 1;
            obj.l_d = A + B*tanh(k*(lobst-C));
               
            % Debug 
            ang1 = fi_d+th_e;
            obj.Pdes = Pe(1:2) + l_d.*[cos(ang1);sin(ang1)];
            ang2 = fi_ei+th_e;
            obj.Pdes_2 = Pe(1:2) + l_ei.*[cos(ang2);sin(ang2)];

            
%%          Informações de Plot
            obj.plotInfo.l = [obj.plotInfo.l (l_d-l_ei)];
            obj.plotInfo.l_ei = [obj.plotInfo.l_ei l_ei];
            obj.plotInfo.r_p = [obj.plotInfo.r_p obj.l_obst];
            obj.plotInfo.r_p_min = [obj.plotInfo.r_p_min (escoltado.Rs*tan(alpha))];
            
%             obj.plotInfo.l_d  = [obj.plotInfo.l_d abs(l_ei-obj.l_d)];
            obj.plotInfo.fi = [obj.plotInfo.fi (mod(fi_d-fi_ei+pi,2*pi)-pi)];
            obj.plotInfo.fi_ei = [obj.plotInfo.fi_ei fi_ei];

%             obj.plotInfo.fi_ei = [obj.plotInfo.fi_ei (mod(fi_ei+pi,2*pi)-pi)];
            
%             obj.plotInfo.fi_d  = [obj.plotInfo.fi_d abs(fi_ei-fi_d)];
%             obj.plotInfo.gamma_ei = [obj.plotInfo.gamma_ei gamma_ei];
%             obj.plotInfo.p_ei = [obj.plotInfo.p_ei p_ei];
%             obj.plotInfo.q_ei = [obj.plotInfo.q_ei q_ei];
            obj.plotInfo.tempos  = [obj.plotInfo.tempos tempo];