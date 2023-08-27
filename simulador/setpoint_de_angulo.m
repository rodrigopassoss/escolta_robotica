if i > 1
    Pdes_ =[];
    P_ =[];
    for rb = Robos
        Pdes_ = [Pdes_ robo(rb).Pdes];
        P_ = [P_ robo(rb).Pdes_2];
    end
    [atribuicoes] = obter_melhores_setpoints(Pdes_,P_);
    fi_d(Robos) = angConvert(2*pi*(atribuicoes-1)/n);
    if atribuicoes~=[1:n]
        atribuicoes
    end
else
    fi_d(Robos) = angConvert(2*pi*[0:n-1]/n);
end