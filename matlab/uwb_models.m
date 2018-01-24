clear all; clc
syms x y r uwb_mux uwb_muy

uwb_sig = 2.4;
uwb_mu1 = [0, 0];
uwb_mu2 = [2, 1];
uwb_mu3 = [-1, 0];
uwb_likelihood = (1/(sqrt(2*pi*uwb_sig)))*exp(-((x-uwb_mux)^2+(y-uwb_muy)^2 - r^2)^2/(2*uwb_sig^2));
obs1 = subs(uwb_likelihood, [r uwb_mux uwb_muy], [3 uwb_mu1]);
obs2 = subs(uwb_likelihood, [r uwb_mux uwb_muy], [5 uwb_mu2]);
obs3 = subs(uwb_likelihood, [r uwb_mux uwb_muy], [2 uwb_mu3]);
fsurf(obs1*obs2*obs3)

%% dsf 

figure()
fc = fcontour(obs1);
fc.LevelList = [0.6];