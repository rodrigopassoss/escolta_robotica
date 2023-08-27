
A = 32; B = 5; dt = 0.01;
duracao = 30; N = duracao/dt;
t = 0:dt:(duracao-dt);
u = zeros(1,N); y = u;
u(1:1000) = 1;
y(1) = dt*A*u(1);
for i = 2:N
y(i) = (1-dt*B)*y(i-1) + dt*A*u(i);
end

plot(t,y)
