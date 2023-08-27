% verifica se � permitido conectar 2 n�s
function [r] = collision_detection(obst,p2,p1,raio_robo)
    % Verifica se a liga��o cruza com o obst�culos
    % Isso � feito usando o produto vetorial 
    if length(obst)~=0
        vet = p2 - p1;
        tam = norm(vet);
        step = tam/50;
        passo = step;
        r = 0;
        while passo < tam
            vet_aux = (vet/norm(vet))*passo;
            p3 = p1 + vet_aux;
            d = sqrt((obst(:,1) - p3(1)).^2 + (obst(:,2) - p3(2)).^2);
            passo = passo + step;
            menor = min(d);
            if ((menor < raio_robo))
               r = 1;
               break;
            end
        end
    else
       r = 0 
    end
end