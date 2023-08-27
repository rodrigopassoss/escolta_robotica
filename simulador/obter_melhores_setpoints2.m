function [atribuicoes] = obter_melhores_setpoints2(fi_d,fi)
    N = length(fi_d);
    A = zeros(N,N);
    caminhos = zeros(N*N,N);
    estadosAnteriores = zeros(N,N);
    
    A(1,:) = abs(mod(fi_d(1)-fi+pi,2*pi)-pi)';
%     A(1,:) = abs(fi_d(1)-fi)';
    for i = 2:N
       for j = 1:N
          [~,bloqueados] = find(caminhos(N*(i-2) + j,:)==1);
          U = setdiff(1:N,unique([bloqueados,j]));
          % Verifica se j já pertence a um dos subcaminhos
          aux = find(caminhos(N*(i-2) + U,j)==1); 
          U = setdiff(U,U(aux));
          [val,idc] = min(A(i-1,U)); 
          if isempty(idc), val=Inf; end 
%           A(i,j) = abs(fi_d(i)-fi(j))'...
%                  + val;
          A(i,j) = abs(mod(fi_d(i)-fi(j)+pi,2*pi)-pi)'...
                 + val;
          if ~isempty(idc)
              caminhos(N*(i-1) + j,U(idc)) = 1;
              estadosAnteriores(i,j) = U(idc);
              aux = j;
              for k = (i-1):-1:1
                  l = estadosAnteriores(k+1,aux);
                  if l==0, break; end
                  if (estadosAnteriores(k,l))~=0
                    caminhos(N*(i-1) + j,estadosAnteriores(k,l)) = 1;
                  end
                  aux = l;
              end
          end
       end
    end
    
    [~, I] = min(A(end,:));
    atribuicoes = I; aux = I;
    for i = (N-1):-1:1
        l = estadosAnteriores(i+1,aux);
        atribuicoes = [atribuicoes l];
        aux = l;
    end
    atribuicoes = atribuicoes(end:-1:1);
end