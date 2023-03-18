function A = updateMapa(Mapa,robo,nRobos)
    %updateMapa Summary of this function goes here
    Bposx = []; % Posição x do corpo do robô
    Bposy = []; % Posição y do corpo do robô
    for k = 1:nRobos
        Bposx = [Bposx round(robo(k).Pos(1) + robo(k).raio*cos(2*pi*[0:99]/100))];
        Bposy = [Bposy round(robo(k).Pos(2) + robo(k).raio*sin(2*pi*[0:99]/100))];
        if ((Bposx(end)<=0)||(Bposy(end)<=0))
            display('pause ... Error');
            pause
        end
    end
    N = length(Bposx);
    A = Mapa;
    for k = 1:N
        if ((~isnan(Bposx(k)))||(~isnan(Bposy(k))))
        A(Bposy(k),Bposx(k)) = 0;
        end
    end
end

