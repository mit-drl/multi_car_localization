function dy = dubins(t, y, v, w, ut)

vt = interp1(ut,v,t);
wt = interp1(ut,w,t);

dy = zeros(3,1);
dy(1) = vt*cos(y(3));
dy(2) = vt*sin(y(3));
dy(3) = wt;

end

