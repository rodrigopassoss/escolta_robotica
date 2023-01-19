function funcao_plotar_caminho_robo(arquivo,nRobos)

load(arquivo);

g = figure(2);
set(g,'name','Resultado final - trajet�ria executada pelos rob�');
axis equal
hold on
formX = [];formY = [];
for k = 1:nRobos
    if nRobos <= 3
        plot(robo(k).plotInfo.P(1,:),robo(k).plotInfo.P(2,:),robo(k).colors(3+k));
    else
        plot(robo(k).plotInfo.P(1,:),robo(k).plotInfo.P(2,:),'b');        
    end
    x = robo(k).plotInfo.P(1,end);
    y = robo(k).plotInfo.P(2,end);
    theta = robo(k).plotInfo.P(3,end);


    % calculo dos pontos de interesse da plotagem
    aux = linspace(0,2*pi*100,100);
    vlimitex = cos(aux)';
    vlimitey = sin(aux)';
    xc = (vlimitex.*robo(k).raio);
    yc = (vlimitey.*robo(k).raio);


    pxyc = [cos(theta) -sin(theta) ; sin(theta) cos(theta)]*[xc' ; yc'];
    xc3 = pxyc(1,:)+x;
    yc3 = pxyc(2,:)+y;
    plot(xc3,yc3,'b')
    plot([x x+robo(k).raio*cos(theta)],[y y+robo(k).raio*sin(theta)],'r');
    
    formX = [formX robo(k).Pos(1)];     
    formY = [formY robo(k).Pos(2)]; 

end
plot([formX formX(1)],[formY formY(1)],'--b');

plot(escoltado.plotInfo.P(1,:),escoltado.plotInfo.P(2,:),'k')
x = escoltado.plotInfo.P(1,end);
y = escoltado.plotInfo.P(2,end);
theta = escoltado.plotInfo.P(3,end);


% calculo dos pontos de interesse da plotagem
aux = linspace(0,2*pi*100,100);
vlimitex = cos(aux)';
vlimitey = sin(aux)';
xc = (vlimitex.*escoltado.raio);
yc = (vlimitey.*escoltado.raio);


pxyc = [cos(theta) -sin(theta) ; sin(theta) cos(theta)]*[xc' ; yc'];
xc3 = pxyc(1,:)+x;
yc3 = pxyc(2,:)+y;
plot(xc3,yc3,'k')
plot([x x+escoltado.raio*cos(theta)],[y y+escoltado.raio*sin(theta)],'r');

[linhas , colunas] = size(A);
xlim([-30 colunas+30])
ylim([-30 linhas+30]) 
plot(Ax,Ay,'.','MarkerEdgeColor','k','MarkerSize',1)
plot(Pdes(1),Pdes(2),'.','MarkerEdgeColor','r','MarkerSize',20)
set(gca,'xtick',[],'ytick',[])

drawnow