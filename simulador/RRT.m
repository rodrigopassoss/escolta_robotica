function [A, Path] = RRT(C_obs,q_init, q_goal, passo, sMap, rRobo)

    q_curr = q_init(1:2);       % Atribui q_init como nó atual
    A = [q_curr, 0];  % Árvore
    flag = 1;              % Bandeira para a finalização do algorítmo
    k = 0;
    while (flag)
        k = k + 1;
       % Gera um posição aleatória 
       q_rand = rand(1,2).*sMap;
       % Obtém o visinho mais próximo
       [~, i_near]= min(((q_rand(1)-A(:,1)).^2 + (q_rand(2)-A(:,2)).^2).^0.5);
       q_near = A(i_near,1:2);
       % Cálcula o avanço da árvore
       q_step = passo.*(q_rand-q_near)./norm(q_rand-q_near);
       % Atualiza o nó atual
       q_curr = q_near + q_step;
       % Verifica se o novo nó 
       [dObs, ~]= min(((q_curr(1)-C_obs(1,:)).^2 + (q_curr(2)-C_obs(2,:)).^2).^0.5);
       [dThree, ~]= min(((q_curr(1)-A(:,1)).^2 + (q_curr(2)-A(:,2)).^2).^0.5);
       if ( (rRobo < dObs) && (dThree==passo) && (~collision_detection(C_obs',q_near,q_curr,rRobo)))
             % Adiciona na àrvore
             A = [A; [q_curr,i_near]]; 
             if (norm(q_curr-q_goal(1:2)) < passo)
                 flag = 0; 
                 [N,~]= size(A);
                 A = [A; [q_goal(1:2),N]];
             end
       end
       
    end
    
    % Obtenção do Caminho
    [N,~]= size(A); i_curr = N;
    Path = A(i_curr,1:2);
    while (i_curr~=1)
         i_curr = A(i_curr,end);
         Path = [Path; A(i_curr,1:2)];
    end
%     Path = Path(end:-1:1,:);
    
end