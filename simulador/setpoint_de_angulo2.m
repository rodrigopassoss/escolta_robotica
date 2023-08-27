fi_d(Robos) = angConvert(2*pi*[0:n-1]/n);
if i > 1
    fi_ =[];
    for rb = Robos
        fi_ = [fi_ robo(rb).X_c(2)];
    end
    [atribuicoes] = obter_melhores_setpoints2(fi_d(Robos),fi_);
    fi_d(Robos(atribuicoes)) = fi_d(Robos);
    if sum(atribuicoes~=[1:n])
        atribuicoes
    end
end