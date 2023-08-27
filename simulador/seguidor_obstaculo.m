function [V,W] = seguidor_obstaculo(v_sensor)

    
    N = 360/length(v_sensor);
    ys_e = 0.5*(v_sensor(90/N - 2)) + (v_sensor(90/N + 2));         % Lado esquerdo
    ys_d = 0.5*(v_sensor(270/N - 2)) + (v_sensor(270/N + 2));       % Lado direito

    %%%% Estimação das variáveis de estado
    % Offset
    d_wall = 30; 
    [d_,I]=min([ys_e;ys_d]); 
    if   I==1, d_ = d_wall - d_;       % Mais próximo pelo lado esquerdo
    else       d_ = d_ - d_wall; end   % Mais próximo pelo lado direito


    % erro angular
    % mean1 = mean(v_sensor(v_sensor<d_wall));
    % mean2 = mean([v_sensor(1:4),v_sensor(end:-1:end-4)]);
    % m = min([mean1,mean2]);
    [~,i_min] = min(v_sensor);
    if   I==1, fi = ((90/N) -i_min);    
    else       fi = ((270/N)-i_min); end 

    fi = 5*fi*(pi/180);  % Transoforma em radianos


    % %%% Controlador
    v_d = 30;  % 35 cm/s
    k1 = 0.44; k2 = 0.008;
    a1 = 1; a2 = a1;
    k_fi = k1/(a1 + abs(fi));
    k_d  = k2/(a2 + abs(d_));

    if(v_sensor(1)<d_wall)
        if I== 1, fi=pi/2; end
        if I== 2, fi=-pi/2; end
    end
    %% Cáculo das velocidades linear (V) e angular (W) do robô (Para o seguidor de Parede)
    k = 0.2; 
    v_c = min([v_d, k*abs(d_)]);
    V = (v_d - v_c)*abs(cos((1-k)*fi));
    %V = v_d 
    wmax = 3.5;
    W = wmax*tanh(-k_fi*fi - k_d*d_*V*sinc(fi));
    
end