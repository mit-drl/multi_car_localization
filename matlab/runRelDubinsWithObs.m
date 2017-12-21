clear all; close all;

v_var = 0.5;
w_var = 0.1;
o_var = 0.05;

max_t = 10;
v1 = [0.4 0.2];
w1 = [-0.3 0.3];
v2 = [0.1 0.5];
w2 = [0.3 -0.3];
ut = [1 max_t];

% set the model's evaluation time points
% (this would be whenever the model
% receives measurement data)
% I am assuming data arrives at
% f Hz and that there is a drop chance
% of a given time point's measurement
% being dropped/not going through
f = 20;
drop = 0.5;
tspan = [];
for i = 1:(1/f):max_t
    if rand() > drop
        tspan = [tspan i];
    end
end

% the dt's are used in the variance calculation
dt = [0];
for i=2:length(tspan)
    dt = [dt tspan(i) - tspan(i-1)];
end

opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
[rel_t, rel_y] = ode45(@(t,y) relative_dubins(t,y,v1,w1,v2,w2,ut), tspan, [1;1;0], opts);

% simulate the cars separately using the dubins model
[c1t, car1] = ode45(@(t,y) dubins(t,y,v1,w1,ut), tspan,[0;0;0], opts);
[t, car2] = ode45(@(t,y) dubins(t,y,v2,w2,ut), tspan,[1;1;0], opts);
car1 = interp1(c1t, car1, t);

% interpolate the transform results into car2's time vector
% (choice of using car2's time vector was arbitrary)
rel_y = interp1(rel_t, rel_y, t);


% need to rotate the transform out of car1's frame and into
% the global frame in order to plot it
for i=1:length(rel_y)
    rotxy = rot(car1(i,3))*[rel_y(i,1); rel_y(i,2)];
    
    rtx(i) = car1(i,1) + rotxy(1);
    rty(i) = car1(i,2) + rotxy(2);
end

% observations
obs_v1 = interp1(ut,v1,rel_t);
obs_w1 = interp1(ut,w1,rel_t);
obs_v2 = interp1(ut,v2,rel_t);
obs_w2 = interp1(ut,w2,rel_t);
for i=1:length(rel_y)
    if i == 1
        dt = 0;
    else
        dt = rel_t(i) - rel_t(i-1);
    end
    obs_v1(i) = obs_v1(i) + normrnd(0,v_var)*dt;
    obs_w1(i) = obs_w1(i) + normrnd(0,o_var)*dt;
    obs_v2(i) = obs_v2(i) + normrnd(0,v_var)*dt;
    obs_w2(i) = obs_w2(i) + normrnd(0,o_var)*dt;
end

[obs_t, obs_y] = ode45(@(t,y) relative_dubins(t,y,obs_v1, ...
    obs_w1,obs_v2,obs_w2,rel_t), tspan, [1;1;0], opts);
obs_y = interp1(obs_t, obs_y, t);
true_o = [];
obs_o = [];
for i=1:length(obs_y)
    o = sqrt(rel_y(i,1)^2 + rel_y(i,2)^2);
    true_o = [true_o o];
    ob_o = sqrt(obs_y(i,1)^2 + obs_y(i,2)^2);
    obs_o = [obs_o ob_o + normrnd(0, o_var)];
end

for i=1:length(obs_y)
    rotxy = rot(car1(i,3))*[obs_y(i,1); obs_y(i,2)];
    
    obsx(i) = car1(i,1) + rotxy(1);
    obsy(i) = car1(i,2) + rotxy(2);
end

figure()
hold on
plot(rel_y(:,1),rel_y(:,2))
plot(obs_y(:,1),obs_y(:,2), 'ro')
hold off

figure()
hold on
plot(car1(:,1), car1(:,2))
% plot(tx, ty, 'bo')
plot(rtx, rty, 'bo')
plot(obsx, obsy, 'ro')
plot(car2(:,1), car2(:,2))
hold off

figure()
hold on
plot(t, true_o);
plot(t, obs_o, 'bo');
hold off