max_t = 10;
v = [1 1];
w = [0.1 0.5];
ut = [1 max_t];
tspan = [1 10];

opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
[t, y] = ode45(@(t,y) dubins(t,y,v,w,ut), tspan, [0;0;0], opts);
plot(y(:,1),y(:,2))