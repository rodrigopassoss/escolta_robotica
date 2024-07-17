function funcao_plotar_graficos(arquivo,nRobos)

load(arquivo);

if habilitaDinamica
    legenda=[];     rpp = [];
    for k = setdiff(1:nRobos,robosComFalhas)
        g = figure(30+k);
        set(g,'name',['Evolução no tempo das velocidades linear e angular do robô ',num2str(k)]);
        subplot(211)
        plot(tempo,robo(k).plotInfo.Pvel(1,:))
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pvel_medido(1,:),'r')
        xlabel('tempo em segundos')
        ylabel('V [cm/s]')
        legend('desejado','medido')
        subplot(212)
        % plot(tempo,Pvel(1,:)*180/pi)
        plot(tempo,robo(k).plotInfo.Pvel(2,:))
        hold on; grid on
        % plot(tempo,Pvel_medido(1,:)*180/pi,'r')
        plot(tempo,robo(k).plotInfo.Pvel_medido(2,:),'r')
        xlabel('tempo em segundos')
        ylabel('W [graus/s]')
        legend('desejado','medido')
% 
%         g2 = figure(40+k);
%         set(g2,'name',['Evolução no tempo da configuração do robô ',num2str(k)])
%         subplot(311)
%         plot(tempo,robo(k).plotInfo.P(1,:))
%         hold on; grid on
%         xlabel('tempo em segundos')
%         ylabel('x [cm]')
%         subplot(312)
%         plot(tempo,robo(k).plotInfo.P(2,:))
%         hold on; grid on
%         xlabel('tempo em segundos')
%         ylabel('y [cm]')
%         subplot(313)
%         % plot(tempo,P(3,:)*180/pi)
%         plot(tempo,robo(k).plotInfo.P(3,:))
%         hold on; grid on
%         xlabel('tempo em segundos')
%         ylabel('theta [graus]')

        g3 = figure(50+k);
        set(g3,'name',['Evolução no tempo das ações de controle Ud e Ue do robô ',num2str(k)]);
        subplot(211)
        plot(tempo,robo(k).plotInfo.Pu(1,:))
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pu_real(1,:))
        xlabel('tempo em segundos')
        ylabel('U_d [-1 , 1]')
        legend('desejado','medido')
        subplot(212)
        plot(tempo,robo(k).plotInfo.Pu(2,:))
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pu_real(2,:))
        xlabel('tempo em segundos')
        ylabel('U_e [-1 , 1]')
        legend('desejado','medido')

        g4 = figure(60+k);
        set(g4,'name',['Evolução no tempo das velocidades das rodas Fid e FIe',num2str(k)]);
        subplot(211)
        plot(tempo,robo(k).plotInfo.Pfi(1,:))
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pfi_real(1,:))
        xlabel('tempo em segundos')
        ylabel('FI_d')
        legend('desejado','medido')
        subplot(212)
        plot(tempo,robo(k).plotInfo.Pfi(2,:))
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pfi_real(2,:))
        xlabel('tempo em segundos')
        ylabel('FI_e')
        legend('desejado','medido')
        
        g2 = figure(40);
        set(g2,'name',['Evolução no tempo das velocidades linear e angular dos robôs ']);
        subplot(211)
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pvel_medido(1,:),robo(k).colors(k),'linewidth',2)
        xlabel('tempo em segundos')
        ylabel('V [cm/s]')
        legenda = [legenda;['robô',num2str(k)]];
        legend(legenda);
        subplot(212)
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pvel_medido(2,:),robo(k).colors(k),'linewidth',2)
        xlabel('tempo em segundos')
        ylabel('W [rad/s]')
        legend(legenda);
        
        
        g5 = figure(50);
        set(g5,'name',['Evolução no tempo das variaveis l^{d}_{ei}-l_{ei} e fi^d_{ei}-f_{ei} dos robôs']);
        subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,experimento.l_d(2:end),'--k')
        xlabel('tempo em segundos')
        ylabel('l^{d}_{ei}-l_{ei}')
        legend(legenda);
        subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('fi^d_{ei}-f_{ei}')
        legend(legenda)
        
        g6 = figure(60);
        set(g6,'name',['Evolução no tempo das variaveis l e fi dos robôs']);
        subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_ei,robo(k).colors(k),'linewidth',2)
        xlabel('tempo em segundos')
        ylabel('l_{ei}')
        legend(legenda);
        subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_ei,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('fi_{ei}')
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
        g7 = figure(70);
        set(g7,'name',['Evolução no tempo das variaveis r_p e r_e dos robôs']);
        subplot(311)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.r_p,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,experimento.l_d(2:end),'--k')
        xlabel('tempo em segundos')
        ylabel('l_{obst}')
        legend(legenda);

        rpp = [rpp;robo(k).plotInfo.r_p];
    end
        g7 = figure(70);
        subplot(312)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,min(rpp),'b','linewidth',2)
        plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.r_p_min.^0)*Rs*(sqrt(2*(1-cos(pi/nRobos)))),'--k','linewidth',1)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('r_p')
        subplot(313)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,experimento.re_d,'b','linewidth',2)
        plot(robo(k).plotInfo.tempos,escoltado.Rs*(experimento.re_d).^0,'--k','linewidth',1)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('r_e')
else
    
    legenda = [];
    rpp = []
    for k = setdiff(1:nRobos,robosComFalhas)
%     for k = robosComFalhas
        
        g = figure(30);
        set(g,'name',['Evolução no tempo das velocidades linear e angular dos robôs ']);
        subplot(211)
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pvel_medido(1,:),robo(k).colors(k),'linewidth',2)
        xlabel('tempo em segundos')
        ylabel('V [cm/s]')
        legenda = [legenda;['robô',num2str(k)]];
        legend(legenda);
        subplot(212)
        hold on; grid on
        plot(tempo,robo(k).plotInfo.Pvel_medido(2,:),robo(k).colors(k),'linewidth',2)
        xlabel('tempo em segundos')
        ylabel('W [rad/s]')
        legend(legenda);
%         legend('desejado','medido')

%         g2 = figure(40+k);
%         set(g2,'name',['Evolução no tempo da configuração do robô ',num2str(k)])
%         subplot(311)
%         plot(tempo,robo(k).plotInfo.P(1,:))
%         hold on; grid on
%         xlabel('tempo em segundos')
%         ylabel('x [cm]')
%         subplot(312)
%         plot(tempo,robo(k).plotInfo.P(2,:))
%         hold on; grid on
%         xlabel('tempo em segundos')
%         ylabel('y [cm]')
%         subplot(313)
%         % plot(tempo,P(3,:)*180/pi)
%         plot(tempo,robo(k).plotInfo.P(3,:))
%         hold on; grid on
%         xlabel('tempo em segundos')
%         ylabel('theta [graus]')
        
        
        g5 = figure(50);
        set(g5,'name',['Evolução no tempo das variaveis l^{d}_{ei}-l_{ei} e fi^d_{ei}-f_{ei} dos robôs']);
        subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,experimento.l_d(2:end),'--k')
        xlabel('tempo em segundos')
        ylabel('l^{d}_{ei}-l_{ei}')
        legend(legenda);
        subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('fi^d_{ei}-f_{ei}')
        legend(legenda)
        
        g6 = figure(60);
        set(g6,'name',['Evolução no tempo das variaveis l e fi dos robôs']);
        subplot(211)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_ei,robo(k).colors(k),'linewidth',2)
        xlabel('tempo em segundos')
        ylabel('l_{ei}')
        legend(legenda);
        subplot(212)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_ei,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('fi_{ei}')
        legend(legenda)

        g7 = figure(70);
        set(g7,'name',['Evolução no tempo das variaveis r_p e r_e dos robôs']);
        subplot(311)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.l_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,robo(k).plotInfo.r_p,robo(k).colors(k),'linewidth',2)
%         plot(robo(k).plotInfo.tempos,experimento.l_d(2:end),'--k')
        xlabel('tempo em segundos')
        ylabel('l_{obst}')
        legend(legenda);

        rpp = [rpp;robo(k).plotInfo.r_p];
    end
        g7 = figure(70);
        subplot(312)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,min(rpp),'b','linewidth',2)
        nn = [((1:650).^0)*6, ((651:3500).^0)*5, ((3501:5962).^0)*4];
        ee = escoltado.Rs.*(sqrt(2.*(1-cos(pi./nn))));
        plot(robo(k).plotInfo.tempos,ee,'--k','linewidth',1)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('r_p')
        subplot(313)
%         plot(robo(k).plotInfo.tempos,robo(k).plotInfo.fi_d)
        hold on; grid on
        plot(robo(k).plotInfo.tempos,experimento.re_d,'b','linewidth',2)
        plot(robo(k).plotInfo.tempos,escoltado.Rs*(experimento.re_d).^0,'--k','linewidth',1)
%         plot(robo(k).plotInfo.tempos,(robo(k).plotInfo.tempos.^0).*experimento.fi_d(k),'--k')
        xlabel('tempo em segundos')
        ylabel('r_e')

    
end