function dy = relative_dubins(t, y, v1, w1, v2, w2, ut)

v1t = interp1(ut,v1,t);
w1t = interp1(ut,w1,t);
v2t = interp1(ut,v2,t);
w2t = interp1(ut,w2,t);

dy = zeros(3,1);
dy(1) = v2t*cos(y(3));
dy(2) = v2t*sin(y(3));
dy(3) = w2t;

% htm = [cos(-w1t) -sin(-w1t) v1t*cos(-w1t);
%        sin(-w1t)  cos(-w1t) v1t*sin(-w1t);
%                0          0             0];
% 
% dy = dy + htm*[y(1); y(2); 0];

r = sqrt(y(1)^2 + y(2)^2);
phi = atan2(y(2),y(1));

tens = [0    w1t;
        -w1t   0];
tens*[y(1); y(2)];

dy(1) = dy(1) + y(2)*(w1t);
dy(2) = dy(2) + y(1)*(-w1t);

% dy(1) = dy(1) + y(2)*cos(-w1t);
% dy(2) = dy(2) + y(1)*sin(-w1t);

% dy(1) = dy(1) + r*cos(-w1t)*sin(phi);
% dy(2) = dy(2) + r*sin(-w1t)*cos(phi);
dy(3) = dy(3) - w1t;

dy(1) = dy(1) - v1t;
dy(2) = dy(2) ;

% dy(1) = dy(1) - v1t*cos(-w1t);
% dy(2) = dy(2) - v1t*sin(-w1t);


end

