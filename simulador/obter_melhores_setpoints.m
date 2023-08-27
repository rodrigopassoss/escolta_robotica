function [atribuicoes] = obter_melhores_setpoints(Pdes,P)
    N = length(P(1,:));
    A = zeros(N,N);
    
    A(1,:) = sqrt((Pdes(1,1)-P(1,:)).^2 + (Pdes(2,1)-P(2,:)).^2);
    
    for i = 2:N
       for j = 1:N
          A(i,j) = sqrt((Pdes(1,i)-P(1,j)).^2 + (Pdes(2,i)-P(2,j)).^2)...
                 + min(A(i-1,setdiff(1:N,j)));
       end
    end
    [~, I] = min(A(end,:));
    atribuicoes = I;
    for i = (N-1):-1:1
         u = setdiff(1:N,atribuicoes);
        [~, I] = min(A(i,u));
        atribuicoes = [atribuicoes u(I)];
    end
    atribuicoes = atribuicoes(end:-1:1);
end