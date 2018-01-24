clear all; close all;

max_t = 10;
v1 = [0.2 0.2];
w1 = [-0.3 0.3];
v2 = [0.1 0.5];
w2 = [0.3 -0.3];
ut = [1 max_t];
tspan = [1 10];

opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
[rel_t, rel_y] = ode45(@(t,y) relative_dubins(t,y,v1,w1,v2,w2,ut), tspan, [1;1;0], opts);
plot(rel_y(:,1),rel_y(:,2))

[c1t, car1] = ode45(@(t,y) dubins(t,y,v1,w1,ut), tspan,[0;0;0], opts);
[t, car2] = ode45(@(t,y) dubins(t,y,v2,w2,ut), tspan,[1;1;0], opts);
car1 = interp1(c1t, car1, t);


relx = interp1(rel_t,rel_y(:,1),t);
rely = interp1(rel_t,rel_y(:,2),t);
relt = interp1(rel_t,rel_y(:,3),t);

for i=1:length(relx)
    rotxy = rot(car1(i,3))*[relx(i); rely(i)];
    
%     tx(i) = car1(i,1) + relx(i); 
%     ty(i) = car1(i,2) + rely(i); 
    
    rtx(i) = car1(i,1) + rotxy(1);
    rty(i) = car1(i,2) + rotxy(2);
end

figure()
hold on
plot(car1(:,1), car1(:,2))
% plot(tx, ty, 'bo')
plot(rtx, rty, 'bo')
plot(car2(:,1), car2(:,2))
hold off