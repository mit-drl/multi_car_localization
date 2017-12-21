function p = transFunc(T_n, T, u, b, dt)
% T_n is the proposed next transform [x_n, y_n, theta_n]
% T is the current transform [x, y, theta]
% u is the inputs [v1, w1, v2, w2]
% b is the noise for each input

x_n = T_n(1);
y_n = T_n(2);
theta_n = T_n(3);
x = T(1); y = T(2); theta = T(3);
v1 = u(1); w1 = u(2); v2 = u(3); w2 = u(4);

mu = 0.5*((x - x_n)*cos(theta) + (y - y_n)*sin(theta))/((y - y_n)*cos(theta) - (x - x_n)*sin(theta));
x_star = (x+x_n)/2 + mu*(y - y_n);
y_star = (y+y_n)/2 + mu*(x_n - x);
r_star = sqrt((x-x_star)^2 + (y-y_star)^2);
dtheta = atan2(y_n-y_star,x_n-x_star) - atan2(y-y_star,x-x_star);

v_hat = dtheta/dt * r_star;
w_hat = dtheta/dt;
g_hat = (theta_n - theta)/dt - w_hat;

p = normpdf(v2 - v_hat, 0, b(3))*normpdf(w2 - w_hat, 0, b(4))*normpdf(g_hat, 0, 0.05);

end

