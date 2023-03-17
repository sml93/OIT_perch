function [c,ceq] = objfcnconstraint(x)

% Intrinsic params
len_prop = 0.127;
len_arm = 0.17;
h_ce = 1.2;
len_l = 0.08;
len_e = 0.0795;
len_ul = 0.109;

gamma = rad2deg(x(1))
angle = rad2deg(x(2))
theta = rad2deg(x(3))
len_nozz = x(4)

% Governing eqns
xob = (h_ce-len_e)/tan(gamma)
zob = h_ce-len_e

xbn = x(4)*sin(x(2)-x(3));
zbn = x(4)*cos(x(2)-x(3));

zpoc = 0;
xpoc = 2;

zpos = zob - zbn - zpoc
xpos = xpoc - (xob + xbn)

t = linspace(0,50,1000);

x0 = [0;zpos;16.8*sin(angle-theta);-16.8*cos(angle-theta)];

sol = ode45(@traj,t,x0);
zerofnd = fzero(@(r)deval(sol,r,2),[sol.x(1),sol.x(end)]);



alpha0 = 1.60;
alpha1 = 0;
R = 0.127;
rho = 1.293;
A = pi*R^2;
vi = 5.18;
mg = 1.5*9.81;
theta = x(3) * pi/180;     %% Allocated max at 0deg
alpha = x(2) * pi/180;    %% Allocated max at 50deg
n = 4;

len_springMax = 0.09;
delta_spring = 0.03;
len_arm = 0.17;
d_ce = len_springMax;
delta = R/(d_ce);
gamma = 0.5*(1-(alpha1*(delta^2))) + 0.5*((1-(alpha1*(delta^2)))^2+(alpha0/8*(delta^2)))^0.5;
Fjetx = 0.901613;
Fjetz = 0.637527;


% %  R matrix for body-spring.
R_b = eye(3,3);
R_b(1,1) = cos(theta);
R_b(1,3) = sin(theta);
R_b(3,1) = -sin(theta);
R_b(3,3) = cos(theta);

% % R matrix for body-jet.
R_bj = zeros(3,3);
R_bj(1,1) = 1;
R_bj(2,2) = cos(alpha);
R_bj(2,3) = -sin(alpha);
R_bj(3,2) = sin(alpha);
R_bj(3,3) = cos(alpha);

% % Selecting indi components of mat
x_comp = [1, 0, 0];
y_comp = [0, 1, 0];
z_comp = [0, 0, 1];

% %  Thrust mat
Fthrust_mat = zeros(3,1);
Fthrust_mat(3,1) = x(7);

% %  Ceiling effect mat
Fce_mat = zeros(3,1);
Fce_mat(3,1) = (2*rho*A*(gamma^2)*(vi^2));

% % Spring mat
Fsp_mat = zeros(3,1);
Fsp_mat(3,1) = x(5)*delta_spring;

% % Jet mat
Fjet_mat = zeros(3,1);
Fjet_mat(1,1) = Fjetx;
Fjet_mat(3,1) = Fjetz;

% % Weight mat
mg_mat = zeros(3,1);
mg_mat(3,1) = mg;

% % Friction mat 
Ffric_mat = zeros(3,1);
Ffric_mat(1,1) = x(6)*x(8);
Ffric_mat(2,1) = x(6)*x(8);


% SUMofF = z_comp*(Fthrust_mat) + 4*z_comp*(Fce_mat) + z_comp*(R_bj*Fjet_mat) - 2*z_comp*(R_b*Fsp_mat) - z_comp*(mg_mat) - z_comp*(R_b*Ffric_mat)
SUMofF = z_comp*(Fthrust_mat) + 4*z_comp*(Fce_mat) + z_comp*(R_bj*Fjet_mat) - 4*z_comp*(R_b*Fsp_mat) - z_comp*(mg_mat) - z_comp*(R_b*Ffric_mat)

SUMofM = z_comp*(Fthrust_mat)*(len_arm*cos(theta)) + 4*z_comp*(Fce_mat)*(len_arm*cos(theta)) + z_comp*(R_bj*Fjet_mat)*(len_arm*cos(theta)) - 2*z_comp*(R_b*Fsp_mat)*(len_arm*cos(theta)) - z_comp*(R_b*Ffric_mat)*(len_arm*cos(theta))





ceq = [xpos - (deval(sol,zerofnd,1)); SUMofM];
c = [-zpos+1; -xpos+(tan(x(2)-x(3))*zpos); SUMofF;]; %z pos of drone must be at least 2m above shelter
% x pos of drone cannot be 2m away, but also cannot be less than 0.2m away.

