% [vlimitex,vlimitey] = scircle2(0,0,1,0); % circulo limite de vis?o MATLAB
aux = linspace(0,2*pi*100,100);
vlimitex = cos(aux)';
vlimitey = sin(aux)';

hold off  
plot(0,0,'.')
hold on
plot(Ax,Ay,'sk','MarkerEdgeColor','k','MarkerSize',2);
% image(A)
% colormap(gray(2))
hold on
axis equal;
formX = [];formY = [];
escoltado = escoltado.plotConfig(Re);
for k = 1:nRobos    

        robo(k) = robo(k).plotConfig(k);
        % plota o destino no ambiente de navegação
        plot(Pdes(1),Pdes(2),'.','MarkerEdgeColor','r','MarkerSize',20)
        set(gca,'xtick',[],'ytick',[])        
        xlim([0 size(A,2)])
        ylim([0 size(A,1)])    
        if ~isempty(find(Robos==k))
            formX = [formX robo(k).Pos(1)];     
            formY = [formY robo(k).Pos(2)]; 
        end
        v = robo(k).Ksi_r(1);
        w = robo(k).Ksi_r(3);
        dx = v*cos(robo(k).Pos(3)) - robo(k).L*w*sin(robo(k).Pos(3));
        dy = v*sin(robo(k).Pos(3)) + robo(k).L*w*cos(robo(k).Pos(3));
        quiver(robo(k).Pos(1),robo(k).Pos(2),dx,dy,'r','MaxHeadSize',3,'linewidth',1);
end
plot([formX formX(1)],[formY formY(1)],'--b');
xlabel('x[cm]');
ylabel('y[cm]');
% nome_do_arquivo = ['C:\Users\rodri\Documents\escolta_robotica\simulador\dados_salvos\plot_',num2str(i),'.svg'];
% saveas(gcf, nome_do_arquivo, 'svg');
drawnow