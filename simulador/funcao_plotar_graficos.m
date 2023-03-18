function funcao_plotar_graficos(arquivo,nRobos)

load(arquivo);

if habilitaDinamica
    legenda=[];
    for k = 1:nRobos
        g = figure(30+k);
        set(g,'name',['Evolução no tempo das velocidades linear e angular do robô ',num2str(k)]);
        subplot(211)
        plot(tempo,robo(k).plotInfo.Pvel(1,:))
        hold on
        plot(tempo,robo(k).plotInfo.Pvel_medido(1,:),'r')
        xlabel('tempo em segundos')
        ylabel('V [cm/s]')
        legend('desejado','medido')
        subplot(212)
        % plot(tempo,Pvel(1,:)*180/pi)
        plot(tempo,robo(k).plotInfo.Pvel(2,:))
        hold on
        % plot(tempo,Pvel_medido(1,:)*180/pi,'r')
        plot(tempo,robo(k).plotInfo.Pvel_medido(2,:),'r')
        xlabel('tempo em segundos')
        ylabel('W [graus/s]')
        legend('desejado','medido')

        g2 = figure(40+k);
        set(g2,'name',['Evolução no tempo da configuração do robô ',num2str(k)])
        subplot(311)
        plot(tempo,robo(k).plotInfo.P(1,:))
        hold on
        xlabel('tempo em segundos')
        ylabel('x [cm]')
        subplot(312)
        plot(tempo,robo(k).plotInfo.P(2,:))
        hold on
        xlabel('tempo em segundos')
        ylabel('y [cm]')
        subplot(313)
        % plot(tempo,P(3,:)*180/pi)
        plot(tempo,robo(k).plotInfo.P(3,:))
        hold on
        xlabel('tempo em segundos')
        ylabel('theta [graus]')

        g3 = figure(50+k);
        set(g3,'name',['Evolução no tempo das ações de controle Ud e Ue do robô ',num2str(k)]);
        subplot(211)
        plot(tempo,robo(k).plotInfo.Pu(1,:))
        hold on
        plot(tempo,robo(k).plotInfo.Pu_real(1,:))
        xlabel('tempo em segundos')
        ylabel('U_d [-1 , 1]')
        legend('desejado','medido')
        subplot(212)
        plot(tempo,robo(k).plotInfo.Pu(2,:))
        hold on
        plot(tempo,robo(k).plotInfo.Pu_real(2,:))
        xlabel('tempo em segundos')
        ylabel('U_e [-1 , 1]')
        legend('desejado','medido')

        g4 = figure(60+k);
        set(g4,'name',['Evolução no tempo das velocidades das rodas Fid e FIe',num2str(k)]);
        subplot(211)
        plot(tempo,robo(k).plotInfo.Pfi(1,:))
        hold on
        plot(tempo,robo(k).plotInfo.Pfi_real(1,:))
        xlabel('tempo em segundos')
        ylabel('FI_d')
        legend('desejado','medido')
        subplot(212)
        plot(tempo,robo(k).plotInfo.Pfi(2,:))
        hold on
        plot(tempo,robo(k).plotInfo.Pfi_real(2,:))
        xlabel('tempo em segundos')
        ylabel('FI_e')
        legend('desejado','medido')

        g5 = figure(70);
        set(g5,'name',['Evolução no tempo das variaveis l e fi do robô',num2str(k)]);
        subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_ei)
%         plot(robo(k).plotInfo.tempos,experimento.l_d(2:end),'--k')
        xlabel('tempo em segundos')
        ylabel('l_{ei}-l_{d}')
        legenda = [legenda;['robô',num2str(k)]];
        legend(legenda);
        subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_ei)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('fi_{ei}-fi_{d}')
        legend(legenda)

%         g6 = figure(80+k);
%         set(g6,'name',['Evolução no tempo das variaveis p_ei e q_ei do robô',num2str(k)]);
%         subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.p_ei)
%         xlabel('tempo em segundos')
%         ylabel('p_{ei}')
%         subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.q_ei)
%         xlabel('tempo em segundos')
%         ylabel('q_{ei}')
    %     legend('desejado','medido')
    end
else
    
    legenda = [];
    for k = 1:nRobos
        g = figure(30+k);
        set(g,'name',['Evolução no tempo das velocidades linear e angular do robô ',num2str(k)]);
        subplot(211)
        plot(tempo,robo(k).plotInfo.Pvel_medido(1,:),'b')
        xlabel('tempo em segundos')
        ylabel('V [cm/s]')
        legend('desejado','medido')
        subplot(212)
        plot(tempo,robo(k).plotInfo.Pvel_medido(2,:),'b')
        xlabel('tempo em segundos')
        ylabel('W [graus/s]')
%         legend('desejado','medido')

        g2 = figure(40+k);
        set(g2,'name',['Evolução no tempo da configuração do robô ',num2str(k)])
        subplot(311)
        plot(tempo,robo(k).plotInfo.P(1,:))
        hold on
        xlabel('tempo em segundos')
        ylabel('x [cm]')
        subplot(312)
        plot(tempo,robo(k).plotInfo.P(2,:))
        hold on
        xlabel('tempo em segundos')
        ylabel('y [cm]')
        subplot(313)
        % plot(tempo,P(3,:)*180/pi)
        plot(tempo,robo(k).plotInfo.P(3,:))
        hold on
        xlabel('tempo em segundos')
        ylabel('theta [graus]')
        
        
        g5 = figure(50);
        set(g5,'name',['Evolução no tempo das variaveis l e fi do robô',num2str(k)]);
        subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_ei)
%         plot(robo(k).plotInfo.tempos,experimento.l_d(2:end),'--k')
        xlabel('tempo em segundos')
        ylabel('l_{ei}-l_{d}')
        legenda = [legenda;['robô',num2str(k)]];
        legend(legenda);
        subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_ei)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('fi_{ei}-fi_{d}')
        legend(legenda)

%         g6 = figure(60+k);
%         set(g6,'name',['Evolução no tempo das variaveis p_ei e q_ei do robô',num2str(k)]);
%         subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.p_ei)
%         xlabel('tempo em segundos')
%         ylabel('p_{ei}')
%         subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.q_ei)
%         xlabel('tempo em segundos')
%         ylabel('q_{ei}')
    end


    
end
