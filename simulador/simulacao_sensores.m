%% tratamento dos sensores (antiga função inicio)
    Ps_i = s_i;
    [ymaxA , xmaxA] = size(A);
    smax = janela*(ones(1,length(angs)));
    smax = smax + raio_robo;
    %s é um vetor cujos elementos representam o ponto da leitura máxima (a priori) para
    %cada um dos sensores no sistema de coordenadas do robô
    s = [smax.*cos(angs) ; smax.*sin(angs)]; % vetor no SC do robô (ainda saturado)    
    Ri = [cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]; % matriz de rotação
    %s_i é o vetor s colocado no sistema de coordenadas do ambiente (sem ruído) (ainda saturado)
    s_i = Ri*s;
    s_i(1,:) = s_i(1,:) + Pos(1);
    s_i(2,:) = s_i(2,:) + Pos(2);
    
    pos_sensor = [raio_robo.*cos(angs) ; raio_robo.*sin(angs)];
    pos_sensor = Ri*pos_sensor;
    pos_sensor(1,:) = pos_sensor(1,:) + Pos(1);
    pos_sensor(2,:) = pos_sensor(2,:) + Pos(2);
    
    for k=1:length(angs)
        Vsx = round(linspace(pos_sensor(1,k),s_i(1,k),janela));
        Vsy = round(linspace(pos_sensor(2,k),s_i(2,k),janela));
        Vs = [Vsx ; Vsy];
        for j=1:janela
            if (Vs(1,j)>0 && Vs(1,j)<=xmaxA && Vs(2,j)>0 && Vs(2,j)<=ymaxA) % se a leitura esta dentro do mapa
                if A(Vs(2,j),Vs(1,j)) ~= 255
                    s_i(1,k) = Vs(1,j);
                    s_i(2,k) = Vs(2,j);
                    break;
                end
            end
        end
    end
    %retorna a medição para o sistema de coordenadas do robô
    % O ROBÔ É ORIENTADO PARA O EIXO X.
    Ps_i(1,:) = s_i(1,:) - Pos(1);
    Ps_i(2,:) = s_i(2,:) - Pos(2);
    s = Ri\Ps_i;
    %adiciona ruído na medição (a resolução da medição depende da resolução do
    %mapa)
    s = s + ruido*randn(2,length(angs));
    % sensor com ruido de medição no S.C. do ambiente
    aux = Ri*s;
    s2(1,:) = aux(1,:) + Pos(1);
    s2(2,:) = aux(2,:) + Pos(2);
    
    %% tratamento dos sensores (antiga função FIM)
        
    v_sensor = sqrt( (s2(1,:)-pos_sensor(1,:)).^2 + (s2(2,:)-pos_sensor(2,:)).^2 ); 
    v_colidiu = sqrt( (s_i(1,:)-pos_sensor(1,:)).^2 + (s_i(2,:)-pos_sensor(2,:)).^2 ); % distância real entre o robô e os obstaculos
    ds0 = sort(v_colidiu);  % ds0 é a distância do sensor sem ruído com a menor leitura
    if ds0(1) <= 1 %colisão de pior caso com 
        colidiu = 1;
    end