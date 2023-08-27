function [A, Path] = RRT(C_obs,q_init, q_goal, passo, sMap, rRobo)

    q_curr = q_init(1:2);       % Atribui q_init como n� atual
    A = [q_curr, 0];  % �rvore
    flag = 1;              % Bandeira para a finaliza��o do algor�tmo
    k = 0;
    while (flag)
        k = k + 1;
       % Gera um posi��o aleat�ria 
       q_rand = rand(1,2).*sMap;
       % Obt�m o visinho mais pr�ximo
       [~, i_near]= min(((q_rand(1)-A(:,1)).^2 + (q_rand(2)-A(:,2)).^2).^0.5);
       q_near = A(i_near,1:2);
       % C�lcula o avan�o da �rvore
       q_step = passo.*(q_rand-q_near)./norm(q_rand-q_near);
       % Atualiza o n� atual
       q_curr = q_near + q_step;
       % Verifica se o novo n� 
       [dObs, ~]= min(((q_curr(1)-C_obs(1,:)).^2 + (q_curr(2)-C_obs(2,:)).^2).^0.5);
       [dThree, ~]= min(((q_curr(1)-A(:,1)).^2 + (q_curr(2)-A(:,2)).^2).^0.5);
       if ( (rRobo < dObs) && (dThree==passo) && (~collision_detection(C_obs',q_near,q_curr,rRobo)))
             % Adiciona na �rvore
             A = [A; [q_curr,i_near]]; 
             if (norm(q_curr-q_goal(1:2)) < passo)
                 flag = 0; 
                 [N,~]= size(A);
                 A = [A; [q_goal(1:2),N]];
             end
       end
       
    end
    
    % Obten��o do Caminho
    [N,~]= size(A); i_curr = N;
    Path = A(i_curr,1:2);
    while (i_curr~=1)
         i_curr = A(i_curr,end);
         Path = [Path; A(i_curr,1:2)];
    end
%     Path = Path(end:-1:1,:);
    
end