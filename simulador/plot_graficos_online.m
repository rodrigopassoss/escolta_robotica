% [vlimitex,vlimitey] = scircle2(0,0,1,0); % circulo limite de vis�o MATLAB
aux = linspace(0,2*pi*100,100);
vlimitex = cos(aux)';
vlimitey = sin(aux)';

hold off  
plot(0,0,'.')
plot(Ax,Ay,'sk','MarkerEdgeColor','k','MarkerSize',1);
% image(A)
% colormap(gray(2))
hold on
axis equal;
formX = [];formY = [];
escoltado = escoltado.plotConfig();
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
end
plot([formX formX(1)],[formY formY(1)],'--b');
xlabel('x[cm]');
ylabel('y[cm]')
drawnow