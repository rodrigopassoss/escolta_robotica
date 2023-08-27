    %% Controle Reativo - For�as Virtuais
    dLim = 10;   % dist�ncia cr�tica (que para come�ar a desviar)
    dmin = 5;
    FLim = 0.01; % 1% da "rea��o m�xima"
    s = log(FLim)/log((dmax-dLim)/(dmax-dmin));
    b = 1/((dmax-dmin)^s);

    % Sensor do Lado esquerdo
    [~,u] = find(obj.angs<=pi);
    [dL, I] = min(obj.v_sensor(u));
    alphaL = obj.angs(u(I));
    cl = (dmax > dL);
    Fl = cl*b*(dmax - dL).^s;

    % Sensor do Lado direito
    [~,u] = find(obj.angs>pi);
    [dR, I] = min(obj.v_sensor(u));
    alphaR = obj.angs(u(I));
    cr = (dmax > dR);
    Fr = cr*b*(dmax - dR).^s;            

    % Seja a Admit�ncia mec�nica (Y):
    % Y = A/(s+B)
    % Y = Freal/F => (Freal(k+1) - Freal)/dt = -BFreal + AF
    %             =>  Freal(k+1) = (1-dtB)Freal + dtAF
    A = 32; B = 5; dt = 0.01;
    F = [Fr;Fl];
    F = (1-dt*B)*obj.Freal + dt*A*F;
    obj.Freal = (cl|cr).*F + (1-(cl|cr)).*[FLim;FLim];
%             % Admit�ncia mec�nica (Y) considerando apenas a constante el�stica
%             Y = obj.Wmax;  % baseado na corre��o de pior caso          

    W =  (F(1)-F(2));
    V = -(F(1)*sin(alphaR)+F(2)*sin(alphaL));

    % Debug do Controle de desvio de obst�culo
    if (dL<dLim)|(dR<dLim)
            display('desviar de obst�culo');
    end