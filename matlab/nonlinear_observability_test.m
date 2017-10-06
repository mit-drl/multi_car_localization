% check Nonlinear Observability
clc;clear
syms phi tht psi vx vy vz ba_x ba_y ba_z b_omg_x b_omg_y b_omg_z g
syms a_imu_x a_imu_y a_imu_z omg_imu_x omg_imu_y omg_imu_z
vn=[vx; vy; vz]; % Note: no need to claim vn. it will automatically be a symbolic variable
ba=[ba_x; ba_y; ba_z];
b_omg=[b_omg_x; b_omg_y; b_omg_z];
a_imu=[a_imu_x; a_imu_y; a_imu_z];
omg_imu=[omg_imu_x; omg_imu_y; omg_imu_z];


e3=[0;0;1];
R_nlb=[cos(tht)*cos(psi),       sin(phi)*sin(tht)*cos(psi)-cos(phi)*sin(psi),        cos(phi)*sin(tht)*cos(psi)+sin(phi)*sin(psi);
    cos(tht)*sin(psi),        sin(phi)*sin(tht)*sin(psi)+cos(phi)*cos(psi),       cos(phi)*sin(tht)*sin(psi)-sin(phi)*cos(psi);
    -sin(tht),                    sin(phi)*cos(tht),                                              cos(phi)*cos(tht)]; % note: there must be no spaces in the formula. Otherwise, an error will occur
R_bln=transpose(R_nlb); % note: never use A' to get the transpose of A, becasue it automatically take the complex conjugate transpose of A. Usually A is real, so instead you shold use the function: transpose(A).
L_nlb=[1, sin(phi)*tan(tht), cos(phi)*tan(tht);
    0, cos(phi), -sin(phi);
    0, sin(phi)/cos(tht), cos(phi)/cos(tht)];

partial_R_bln_phi=diff(R_bln, phi);
partial_R_bln_tht=diff(R_bln, tht);
partial_R_bln_psi=diff(R_bln, psi);

H=R_bln*vn*e3'*transpose(R_bln);

% state vector ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
x=[vx; vy; vz; phi; tht; psi; ba_x; ba_y; ba_z; b_omg_x; b_omg_y; b_omg_z];

% meausrement model z=h(x) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
h=H(:);

% process model \dot{x}=f(x,u) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
f=[     R_nlb*(a_imu+ba)+g*e3;
         L_nlb*(omg_imu+b_omg);
         0;
         0;
         0;
         0;
         0;
         0];

% zeroth order Lie direvative & its Jacobian wrt x~~~~~~~~~~~~~~~~~~~~~~~~~
ph_px=jacobian(h,x);
ph_px_value=simplify(  subs(ph_px, {phi, tht, psi}, {0,0,0})   ); % correct!!! verified!!!

% first order Lie direvative & its Jacobian wrt x ~~~~~~~~~~~~~~~~~~~~~~~~~
Lfh=ph_px*f;
pLfh_px=jacobian(Lfh, x);
pLfh_px_value=simplify(  subs(pLfh_px, {phi, tht, psi, omg_imu(1), omg_imu(2), omg_imu(3)}, {0,0,0, -b_omg(1), -b_omg(2), -b_omg(3)})   ); % correct!!! verified!!!

% second order Lie direvative & its Jacobian wrt x ~~~~~~~~~~~~~~~~~~~~~~~~
L2fh=pLfh_px*f;
pL2fh_px=jacobian(L2fh, x);
% tic
% pL2fh_px_value=subs(pL2fh_px, {phi, tht, psi, omg_imu}, {0,0,0, -b_omg}); % Elapsed time is 538.652010 seconds.
% toc


% third order Lie direvative & its Jacobian wrt x ~~~~~~~~~~~~~~~~~~~~~~~~~~
L3fh=pL2fh_px*f;
pL3fh_px=jacobian(L3fh, x);
tic
pL3fh_px_value=subs(pL3fh_px, {phi, tht, psi, omg_imu(1), omg_imu(2), omg_imu(3)}, {0,0,0, -b_omg(1), -b_omg(2), -b_omg(3)});
toc